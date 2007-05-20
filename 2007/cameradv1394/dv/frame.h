/*
* This file is taken/modified from the codebase of Kino, a Linux video editor
* which is *known* to talk to our DV camera correctly.
* 
* Used without permission.
*/


/*
* frame.h -- utilities for process digital video frames
* Copyright (C) 2000 Arne Schirmacher <arne@schirmacher.de>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software Foundation,
* Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
*/

#ifndef _FRAME_H
#define _FRAME_H 1

// Comment this line out to disable LIBDV usage.
// (I suspect, however, that if you actually did that,
//  this class would become nonfunctional - although I
//  can't say for certain.)
#define HAVE_LIBDV 1

// Controls the quality of the image received
// from (?) and sent to (?) the camera.
// [1=best, 5=fastest]
#define PREF_DISPLAY_QUALITY 1

// (Kino defaults to both of these values being false)
#define SHOULD_CLAMP_LUMA FALSE
#define SHOULD_CLAMP_CHROMA FALSE

// (I have no idea what this should be set to by default.)
#define USE_TWO_PASS_DV_ENCODER TRUE

#include <iostream>
using std::cerr;
using std::endl;

#include <time.h>
#include <string>
#include <stdio.h>
#include <samplerate.h>

#ifdef HAVE_LIBDV
#include <libdv/dv.h>
#include <libdv/dv_types.h>
#endif

#define FRAME_MAX_WIDTH 720
#define FRAME_MAX_HEIGHT 576

typedef struct Pack
{
	/// the five bytes of a packet
	unsigned char data[ 5 ];
}
Pack;

typedef struct TimeCode
{
	int hour;
	int min;
	int sec;
	int frame;

	bool operator==(const TimeCode& other) {
		return
			(this->hour == other.hour) &&
			(this->min == other.min) &&
			(this->sec == other.sec) &&
			(this->frame == other.frame);
	}
}
TimeCode;


typedef struct AudioInfo
{
	int frames;
	int frequency;
	int samples;
	int channels;
	int quantization;
}
AudioInfo;

class Frame
{
public:
	/// the frame buffer; must be manually updated before frame data is accessed
	unsigned char *data;
	/// the number of bytes written to the frame; should be updated after a new frame is read
	int bytesInFrame;

#ifdef HAVE_LIBDV
	dv_decoder_t *decoder;
	dv_encoder_t *encoder;
#endif

	int16_t *audio_buffers[ 4 ];
	uint8_t *tempImage;

public:
	Frame(int frameSize);
	~Frame();
	
	bool IsValid() const;

	bool GetSSYBPack( int packNum, Pack &pack ) const;
	bool GetVAUXPack( int packNum, Pack &pack ) const;
	bool GetAAUXPack( int packNum, Pack &pack ) const;
	bool GetTimeCode( TimeCode &timeCode ) const;
	bool GetAudioInfo( AudioInfo &info ) const;
	int GetFrameSize( void ) const;
	float GetFrameRate( void ) const;
	bool IsPAL( void ) const;
	bool IsNewRecording( void ) const;
	bool IsNormalSpeed( void ) const;
	bool IsComplete( void ) const;
	int ExtractAudio( void *sound ) const;

#ifdef HAVE_LIBDV
	void SetPreferredQuality( );
	int ExtractAudio( int16_t **channels ) const;
	void ExtractHeader( void );
	void Deinterlace( uint8_t *pdst, uint8_t *psrc, int stride, int height );
	int ExtractRGB( void *rgb );
	int ExtractYUV( void *yuv );
	int ExtractYUV420( uint8_t *yuv, uint8_t *output[ 3 ] );
	void GetUpperField( void *image, int bpp );
	void GetLowerField( void *image, int bpp );
	bool IsWide( void ) const;
	int GetWidth() const;
	int GetHeight() const;
	bool CreateEncoder( bool isPAL, bool isWide );
	void SetRecordingDate( time_t *datetime, int frame );
	void SetTimeCode( int frame );
	bool EncodeAudio( AudioInfo &info, int16_t **channels );
	int CalculateNumberSamples( int frequency, int iteration );
	void EncodeRGB( uint8_t *rgb );
#endif

private:
	/// flag for initializing the lookup maps once at startup
	static bool maps_initialized;
	/// lookup tables for collecting the shuffled audio data
	static int palmap_ch1[ 2000 ];
	static int palmap_ch2[ 2000 ];
	static int palmap_2ch1[ 2000 ];
	static int palmap_2ch2[ 2000 ];
	static int ntscmap_ch1[ 2000 ];
	static int ntscmap_ch2[ 2000 ];
	static int ntscmap_2ch1[ 2000 ];
	static int ntscmap_2ch2[ 2000 ];
	static short compmap[ 4096 ];
};

typedef enum {
    AUDIO_RESAMPLE_SRC_SINC_BEST_QUALITY = 0,
    AUDIO_RESAMPLE_SRC_SINC_MEDIUM_QUALITY = 1,
    AUDIO_RESAMPLE_SRC_SINC_FASTEST = 2,
    AUDIO_RESAMPLE_SRC_ZERO_ORDER_HOLD = 3,
    AUDIO_RESAMPLE_SRC_LINEAR = 4,
    AUDIO_RESAMPLE_INTERNAL = 5
}
AudioResampleType;

#define BUFFER_LEN 20480

template <class input_t, class output_t> class AudioResample
{
protected:
	double output_rate;
	input_t input[ BUFFER_LEN ];

public:
	AudioResample( double rate ) : output_rate( rate )
	{ }
	virtual ~AudioResample()
	{ }
	virtual void Resample( input_t *samples, double input_rate, int channels, int samples_this_frame )
	{ }
	void Resample( Frame &frame )
	{
		//  cout << "Resample -----------------" << endl;
		if ( output_rate != 0 )
		{
			frame.ExtractAudio( input );
			AudioInfo info;
			frame.GetAudioInfo( info );
			/*
			cerr << "Audio info: "
			     << "  dec-..->frequency: "
			     << info.frequency
			     << "  samples: "
			     << info.samples << endl;
 			*/
			if ( info.frequency && output_rate != info.frequency )
			{
				Resample( input,
					  info.frequency,
					  info.channels,
					  info.samples );
			}
			else
			{
				for (int i=0; i<info.channels*info.samples; i++)
					output[i] = input[i];

				size = info.channels*info.samples*sizeof(output_t);
			}
		}
		else
		{
			size = 0;
		}
		// cout << "size: " << size << endl;
	}
	void SetOutputFrequency( double output_rate )
	{
		this->output_rate = output_rate;
	}
	int GetOutputFrequency( )
	{
		return this->output_rate;
	}

	output_t output[ BUFFER_LEN ];
	int size;
};

template <class input_t, class output_t> class InternalAudioResample : public AudioResample<input_t, output_t>
{
public:
	InternalAudioResample( ) : AudioResample<input_t,output_t>( 0 )
	{ }
	InternalAudioResample( double output_rate ) : AudioResample<input_t,output_t>( output_rate )
	{ }
	virtual ~InternalAudioResample()
	{ }
	/** Simple up and down resampler for people unable to handle 48khz
	    audio. Also fixes mixed projects with 32khz and 48khz audio
	    sampling (yippee!).  
	*/
	void Resample( input_t *input, double input_rate, int channels, int samples )
	{
		float ratio = ( float ) this->output_rate / ( float ) input_rate;
		this->size = ( int ) ( ( float ) samples * ratio );

		int rounding = 1 << 15;
		unsigned int xfactor = ( samples << 16 ) / this->size;
		unsigned int xmax = xfactor * this->size;
		unsigned int i = 0;
		unsigned int o = 0;
		this->size *= sizeof(output_t) * channels;

		for ( unsigned int xft = 0; xft < xmax; xft += xfactor )
		{
			i = ( ( xft + rounding ) >> 16 ) * channels;
			for (int c=0; c < channels; c++)
				this->output[o+c] = input[i+c];
			o += channels;
		}

	}
};

template <class input_t, class output_t> class SrcAudioResample : public AudioResample<input_t, output_t>
{
public:
	SrcAudioResample( int converter ) : AudioResample<input_t,output_t>( 0 )
	{
		SrcAudioResample( converter, 0 );
	}
	SrcAudioResample( int converter, double output_rate, bool isStreaming ) :
		AudioResample<input_t,output_t>( output_rate )
	{
		int srcError = 0;

		state = src_new( converter, 2, &srcError );
		if ( srcError != 0 )
		{
			cerr << "SRC: " << src_strerror( srcError ) << endl;
		}
		else
		{
			data.data_in = input_buffer;
			data.data_out = output_buffer;
			data.end_of_input = isStreaming ? 0 : 1;
		}
	}
	virtual ~SrcAudioResample()
	{
		src_delete( state );
	}
	void Resample( input_t *input, double input_rate, int channels, int samples )
	{
		src_short_to_float_array( input, input_buffer, samples * channels );

		// Setup resampler
		data.input_frames = samples;
		data.output_frames = BUFFER_LEN / channels;
		data.src_ratio = this->output_rate / input_rate;

		// Resample
		int result = src_process ( state, &data );
		if ( result != 0 )
			cerr << "SRC: " << src_strerror( result ) << endl;
		this->size = data.output_frames_gen * channels * sizeof(output_t);

// cerr << "Resample in samples " << samples << " out rate " << this->output_rate << " in rate " << input_rate << " src_ratio " << data.src_ratio << "out samples " << this->size/4 << endl;

		src_float_to_short_array( output_buffer, this->output, data.output_frames_gen * channels );
	}

private:
	SRC_STATE *state;
	SRC_DATA data;
	float input_buffer[ BUFFER_LEN ];
	float output_buffer[ BUFFER_LEN ];
};

template <class input_t, class output_t> class AudioResampleFactory
{
public:
	static AudioResample<input_t, output_t> *createAudioResample( AudioResampleType type, 
		double output_rate = 0, bool isStreaming = true )
	{
		switch ( type )
		{
		case AUDIO_RESAMPLE_SRC_SINC_BEST_QUALITY:
			return new SrcAudioResample<input_t, output_t>( SRC_SINC_BEST_QUALITY, output_rate, isStreaming );
		case AUDIO_RESAMPLE_SRC_SINC_MEDIUM_QUALITY:
			return new SrcAudioResample<input_t, output_t>( SRC_SINC_MEDIUM_QUALITY, output_rate, isStreaming );
		case AUDIO_RESAMPLE_SRC_SINC_FASTEST:
			return new SrcAudioResample<input_t, output_t>( SRC_SINC_FASTEST, output_rate, isStreaming );
		case AUDIO_RESAMPLE_SRC_ZERO_ORDER_HOLD:
			return new SrcAudioResample<input_t, output_t>( SRC_ZERO_ORDER_HOLD, output_rate, isStreaming );
		case AUDIO_RESAMPLE_SRC_LINEAR:
			return new SrcAudioResample<input_t, output_t>( SRC_LINEAR, output_rate, isStreaming );
		default:
			return new InternalAudioResample<input_t, output_t>( output_rate );
		}
	}
};

#endif
