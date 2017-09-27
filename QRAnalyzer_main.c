/*
 *QRAnalyzer is an everis AED application that allows read QRs using an
 *AXIS camera.
 * 
 * It uses AXIS SDK to capture camera streaming and at the same time,
 * QR libraries to recognize and decode QRs read by the camera.  

*/

//Axis libreries
#define _GNU_SOURCE
//#include <glib.h>
#include <signal.h>
#include <string.h>
#include <capture.h>
#include <syslog.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/time.h>
#include <glib/gstdio.h>
#include <gio/gio.h>
#include <axsdk/axhttp.h>
#include <axsdk/axaudio.h>
#include <sys/time.h>
#include "quirc.h"

#ifdef DEBUG
#define D(x)    x
#else
#define D(x)
#endif

#define LOGINFO(fmt, args...)    { syslog(LOG_INFO, fmt, ## args); printf(fmt, ## args); }
#define LOGERR(fmt, args...)     { syslog(LOG_CRIT, fmt, ## args); fprintf(stderr, fmt, ## args); }

#define APPNAME "QRAnalyzer"
#define CAPTURE_PROP_FMT "resolution=%dx%d&sdk_format=Y800&fps=%d"

#define CAPTURE_WIDTH   320
#define CAPTURE_HEIGHT  240
#define CAPTURE_FPS     10

#define ERROR 1001
#define ERROR_NEW_QUIRC 1002

#define CAPTURE_FPS     10
#define ITERATION_PERIOD (1000 / CAPTURE_FPS)

#define MAX_LEVEL 10

#define MAX_NUM_CODES 10
#define LENGTH_CODES 50

static struct quirc *decoder;
static struct dimensions
{

	guint width;
        guint height;

}dims;

static struct result
{
       struct quirc_data dataDump;
       time_t clk;
       guint error;

}resul[100];

media_stream * stream;
gchar *capture_properties;
gint iterations=0;
static GMainLoop *loop;
media_frame * frame, *prevframe;
gchar *frameData = NULL;
gchar *prevFrameData=NULL;
struct quirc_data dataDump[500];

gchar * errorList[]={"QUIRC_ERROR_INVALID_GRID_SIZE",
	"QUIRC_ERROR_INVALID_VERSION",
	"QUIRC_ERROR_FORMAT_ECC",
	"QUIRC_ERROR_DATA_ECC",
	"QUIRC_ERROR_UNKNOWN_DATA_TYPE",
	"QUIRC_ERROR_DATA_OVERFLOW",
	"QUIRC_ERROR_DATA_UNDERFLOW"};


//Util to convert string to guint
static gboolean util_string_to_guint(const gchar *str, guint *val)
{
  const gchar *p;
  guint lval;
  gboolean result = FALSE;

  p = str;
  lval = 0;

  while (*p) {
    if (!g_ascii_isdigit(*p)) {
      goto error;
    }

    lval = lval * 10 + (*p - '0');

    p++;
  }

  *val = lval;

  result = TRUE;

error:
  return result;
}

//Function to obtain optimal resolution list
static gchar* util_get_maxres(void)
{
  int len = 0;
  gchar *tmp = NULL;
  gchar *res = NULL;
  char *buf = capture_get_optimal_resolutions_list(1);

  tmp = g_strstr_len(buf, strlen(buf), ",");
  len = strlen(buf);
  if (tmp) {
    len -= strlen(tmp);
  }
  res = g_strndup(buf, len);

  if(buf) {
    free(buf);
  }

  return res;
}

static AXAudioFrame *
generate_sine(gint length, gint frequency, guint sound_level)
{
  guint frame_rate = 16000;
  guint sample_per_sin =  frame_rate / frequency;
  AXAudioFrame *frame = ax_audio_frame_new();
  gint16 *data = g_malloc0(sizeof(gint16) * frame_rate * length);
  gint i = 0;
  for (;i < (frame_rate * length); i++) {
    data[i] = sin((2 * G_PI / ((sample_per_sin)) * i)) * sound_level;
  }
  ax_audio_frame_set_data(frame, (gpointer)data, sizeof(gint16)* frame_rate * length, (GFreeFunc)&g_free, NULL);
  return frame;
}

//Function to detect recomended resolution to camera initialization
static gchar* get_resolution(guint *width, guint *height)
{
  gchar *resolution_str = NULL;
  gchar **resolution_parts = NULL;

  resolution_str = util_get_maxres();

  if (!resolution_str){

    LOGERR("Failed to get resolution\n");
    goto error;

  }

  if(g_strrstr(resolution_str, "x")) {
    resolution_parts = g_strsplit(resolution_str, "x", 0);

    if (!util_string_to_guint(resolution_parts[0], width) ||
        !util_string_to_guint(resolution_parts[1], height)) 
   {

      LOGERR("Failed to parse resolution: %s\n", resolution_str);
      goto error;
    
    }

  } else { /* fallback*/
    *width = CAPTURE_WIDTH;
    *height = CAPTURE_HEIGHT;
  }

error:
  if (resolution_parts != NULL) {
    g_strfreev(resolution_parts);
  }

  return resolution_str;
}

static gint QRAnalyzer (void)
{
  GError *error = NULL;
  uint8_t *image;
  struct quirc_code code;
  gint numframes  = 100;
  AXAudioFrame *audioFrame = NULL;
  AXAudioOutput *output = NULL;


  /*LOGINFO("Getting[%d]: %d frames. resolution: %dx%d framesize: %d stride: %d\n",
          iterations,
          numframes,
          capture_frame_width(frame),
          capture_frame_height(frame),
          capture_frame_size(frame),
          capture_frame_stride(frame));*/
  guint x,y,level=0;
  //LOGINFO("ENTRO\n");
  
  /* print intital information */
  frame = capture_get_frame(stream);

  guint prev_stride = capture_frame_stride(prevframe);
  guint curr_stride = capture_frame_stride(frame);
  frameData = capture_frame_data(frame);
  prevFrameData  = capture_frame_data(prevframe);

  gchar * delta_matrix = g_malloc(sizeof(gchar) * dims.width *
      dims.height);

  /* Poor man's motion detection. Compare and save the difference in pixel
   * values between the current frame and the previous frame.
   */

  if (prevframe == NULL) {

    /* If this is the first time we're called then there's nothing to compare
     * this frame to. Just save the frame and return.
     */
    prevframe = frame;

  }
  else
  {
     
	  for (y = 0; y < dims.height; y++) {
	    guint prev_pos_y = y * prev_stride;
	    guint curr_pos_y = y * curr_stride;
	    guint delta_h    = y * dims.height;

	    for (x = 0; x < dims.width; x++) {
	      guint prev_pos = prev_pos_y + x;
	      guint curr_pos = curr_pos_y + x;

	      delta_matrix[delta_h + x] = ABS(frameData[curr_pos] - prevFrameData[prev_pos]);

	      level+=delta_matrix[delta_h + x];

	    }
	  }

	  level/=dims.width*dims.height;

	  if(level >= MAX_LEVEL)
	  {
                //LOGINFO("HA HABIDO UN MOVIMIENTO[iteration: %d]\n", iterations+1); 
	    	decoder = quirc_new();
		if (!decoder) {
		      LOGERR("QUIRC_NEW");
		      g_free(capture_properties);
		      closelog();
		      return EXIT_FAILURE;
		}
		  
		if (quirc_resize(decoder,(int *) dims.width, (int *) dims.height) < 0){
		      LOGERR("QUIRC_RESIZE");
		      quirc_destroy(decoder);
		      g_free(capture_properties);
		      closelog();
		      return EXIT_FAILURE;
		}

		image=quirc_begin(decoder, NULL, NULL);	

         	for (y = 0; y < dims.height; y++) 
		{
	  		guint curr_pos_y = y * dims.height;
			for (x = 0; x < dims.width; x++) {
		      
		      		guint curr_pos = curr_pos_y + x;
		      		image[curr_pos] = frameData[curr_pos];

                                //LOGINFO("%d, %d, %d, %d,  %X|%X ", y, curr_pos_y, x, curr_pos, frameData[curr_pos], image[curr_pos]);
		    	}
		}

		quirc_end(decoder); 
		guint num_codes,i;
		num_codes = quirc_count(decoder);
		//LOGINFO("NUM_CODES[iteration: %d]: %d\n", iterations+1, num_codes);

		for (i = 0; i < num_codes; i++) {
			struct quirc_code code;
			struct quirc_data data;

			quirc_extract(decoder, i, &code);
                        quirc_decode_error_t err = quirc_decode(&code, &data);
			if (err)
			{
			    LOGINFO("ERROR LECTURA QR[iteration: %d]: %d\n", iterations, err);
			    resul[iterations].error=err;
			    resul[iterations].clk=time(NULL);					
			}
			else
			{
			     LOGINFO("HAY UN QR DETECTADO[iteration: %d]: %s\n", iterations+1, data.payload);	
                             resul[iterations].dataDump=data;
                             resul[iterations].clk=time(NULL);
			     resul[iterations].error=err;

			     audioFrame=generate_sine(2, 2500, 15000);
			     output = ax_audio_open_output_pcm_16(-1, 16000, AX_AUDIO_CHANNEL_MONO, &error);
			  
			     if (!ax_audio_output_play_pcm_16(output, audioFrame, &error)) {
			         //syslog(LOG_WARNING, "Failed to play, code: %d and error message: %s",
				 //error->code, error->message);
			         //goto error_out;
			     }
			     if (!ax_audio_output_close_pcm_16(output, &error)) {
			         //syslog(LOG_WARNING, "Failed to close playback, code: %d and error message: %s",
				 //error->code, error->message);
			         //goto error_out;
			     }
			     
			}

                    	iterations++;
	  	}

		quirc_destroy(decoder);

	  }
	  else
	  {
	    //LOGINFO("LA VIDA SIGUE IGUAL\n");
	  }

	  
	  capture_frame_free(prevframe);
	  prevframe = frame;
          g_free(delta_matrix);
          level=0;
   }

  LOGINFO("SALGO\n");
}

static void
request_handler(const gchar *path,
    const gchar *method,
    const gchar *query,
    GHashTable *params,
    GOutputStream *output_stream,
    gpointer user_data)
{
  GDataOutputStream *dos;
  guint *timer = (guint*) user_data;
  gchar msg[512];
  gint i;

  dos = g_data_output_stream_new(output_stream);

  /* Send out the HTTP response status code */
  g_data_output_stream_put_string(dos,"Content-Type: text/html\r\n", NULL, NULL);
  g_data_output_stream_put_string(dos,"Status: 200 OK\r\n\r\n", NULL, NULL);
  g_snprintf(msg, sizeof(msg),"<script>setTimeout('location.reload(true);',5000);</script>\r\n");
  g_data_output_stream_put_string(dos, msg, NULL, NULL);
  /* Our custom message */
  g_snprintf(msg, sizeof(msg),"<p>- QR ANALYZER - DETECTED CODES -'%s'</p>", path ? path:"(NULL)");
  g_data_output_stream_put_string(dos, msg, NULL, NULL);

  g_snprintf(msg, sizeof(msg),"<hr width=63px>" 
                              "<p>THEESE ARE QR-CODES DETECTED: </p>");
  g_data_output_stream_put_string(dos, msg, NULL, NULL);


  if (iterations != 0)
  {
	  for (i=iterations-1; i>0; i--)
	  {

                  if (resul[i].error == 0)
		  {
			  /*g_snprintf(msg, sizeof(msg), "\n%s() [%s:%d] \n\n"
						       "-  DETECTED CODE -\n"
						       "CODE[iterations: %d]: %s\n",
						       __FUNCTION__, __FILE__, __LINE__, i+1, dataDump[i].payload);*/
			  g_snprintf(msg, sizeof(msg), "<p>-  DETECTED CODE - </p>"
						       "<p>CODE[ITERATION: %d]: %s AT %s </p>",
						       i+1, resul[i].dataDump.payload, ctime(&resul[i].clk));
			  g_data_output_stream_put_string(dos, msg, NULL, NULL);
		 }
                 else
                 {
			g_snprintf(msg, sizeof(msg), "<p>-  DETECTED CODE WITH ERROR -</p>"
						       "<p>ERROR[ITERATION: %d]: %d BECAUSE %s AT %s</p>",
						       i+1, resul[i].error, errorList[resul[i].error-1], ctime(&resul[i].clk));
			g_data_output_stream_put_string(dos, msg, NULL, NULL);

                 }
	  }
  }
  else
  {
	g_snprintf(msg, sizeof(msg),"<p>NOTHING READ - SHOW THE QR-CODE TO THE CAMERA</p>");
  	g_data_output_stream_put_string(dos, msg, NULL, NULL);
  } 
  g_object_unref(dos);
}

//Main function - App Start point
gint main (void){
	
  gchar *res_str = NULL;
  guint source_width;
  guint source_height;
  AXHttpHandler *handler;
  guint timer = 20;

  //detectedCodes = g_malloc(sizeof(gchar) * MAX_NUM_CODES * LENGTH_CODES);
  	
  openlog("QRANALYZER", LOG_PID | LOG_CONS, LOG_USER);

  LOGINFO("%s STARTED...\n", APPNAME);

  LOGINFO("GETTING RESOLUTION\n");
  if (!(res_str=get_resolution(&source_width, &source_height)))
  {
	g_free(res_str);
	goto error;
  }

  capture_properties = g_strdup_printf(CAPTURE_PROP_FMT, source_width, source_height, CAPTURE_FPS );

  stream = capture_open_stream(IMAGE_UNCOMPRESSED, capture_properties);
  if (stream == NULL) {
    LOGERR("Failed to open stream\n");
    g_free(capture_properties);
    closelog();
    return EXIT_FAILURE;
  }

  dims.width=source_width;
  dims.height=source_height;

  handler = ax_http_handler_new(request_handler, &timer);

  g_timeout_add(ITERATION_PERIOD, (GSourceFunc)QRAnalyzer, NULL);

  loop = g_main_loop_new(NULL, FALSE);
  g_main_loop_run(loop);
  
  //QRAnalyzer(source_width, source_height);
  g_main_loop_unref(loop);
  capture_close_stream(stream);
  g_main_loop_unref(loop);
  ax_http_handler_free(handler);
  closelog();

  return EXIT_SUCCESS;	

error:
      
   LOGERR("Failed to open stream\n");
   return ERROR;

}
