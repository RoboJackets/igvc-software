/*
* This file is taken/modified from the codebase of Kino, a Linux video editor
* which is *known* to talk to our DV camera correctly.
* 
* Used without permission.
*/

/*
* frame.cc -- utilities for processing digital video frames
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

/** Code for handling raw DV frame data
 
    These methods are for handling the raw DV frame data. It contains methods for 
    getting info and retrieving the audio data.
 
    \file frame.cc
*/

// C++ includes

#include <string>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <deque>

using std::ostringstream;
using std::setw;
using std::setfill;
using std::hex;
using std::dec;
using std::deque;
using std::cerr;
using std::endl;

// C includes
#include <pthread.h>
#include <math.h>

// local includes
#include "frame.h"
#include "dv1394.h"

#ifndef HAVE_LIBDV
bool Frame::maps_initialized = false;
int Frame::palmap_ch1[ 2000 ];
int Frame::palmap_ch2[ 2000 ];
int Frame::palmap_2ch1[ 2000 ];
int Frame::palmap_2ch2[ 2000 ];

int Frame::ntscmap_ch1[ 2000 ];
int Frame::ntscmap_ch2[ 2000 ];
int Frame::ntscmap_2ch1[ 2000 ];
int Frame::ntscmap_2ch2[ 2000 ];

short Frame::compmap[ 4096 ];
#endif

pthread_mutex_t avcodec_mutex = PTHREAD_MUTEX_INITIALIZER;


/** constructor
 
    All Frame objects share a set of lookup maps,
    which are initalized once (we are using a variant of the Singleton pattern). 
 
*/

Frame::Frame(int frameSize) : bytesInFrame( 0 ), tempImage( 0 )
{
	data = NULL;	// force user to manually allocate/map the frame buffer
	//data = ( unsigned char* ) calloc( 1, frameSize );

#ifdef HAVE_LIBDV
	decoder = dv_decoder_new( FALSE,		// (ignored)
	                          SHOULD_CLAMP_LUMA,
	                          SHOULD_CLAMP_CHROMA );
	// BUGFIX: Make sure initialization was successful
	if (decoder != NULL) {
		decoder->audio->arg_audio_emphasis = 2;
		this->SetPreferredQuality( );
		dv_set_audio_correction ( decoder, DV_AUDIO_CORRECT_AVERAGE );
		dv_set_error_log( decoder, NULL );
	}

	encoder = NULL;
#else

	if ( maps_initialized == false )
	{

		for ( int n = 0; n < 1944; ++n )
		{
			int sequence1 = ( ( n / 3 ) + 2 * ( n % 3 ) ) % 6;
			int sequence2 = sequence1 + 6;
			int block = 3 * ( n % 3 ) + ( ( n % 54 ) / 18 );

			block = 6 + block * 16;
			{
				register int byte = 8 + 2 * ( n / 54 );
				palmap_ch1[ n ] = sequence1 * 150 * 80 + block * 80 + byte;
				palmap_ch2[ n ] = sequence2 * 150 * 80 + block * 80 + byte;
				byte += ( n / 54 );
				palmap_2ch1[ n ] = sequence1 * 150 * 80 + block * 80 + byte;
				palmap_2ch2[ n ] = sequence2 * 150 * 80 + block * 80 + byte;
			}
		}
		for ( int n = 0; n < 1620; ++n )
		{
			int sequence1 = ( ( n / 3 ) + 2 * ( n % 3 ) ) % 5;
			int sequence2 = sequence1 + 5;
			int block = 3 * ( n % 3 ) + ( ( n % 45 ) / 15 );

			block = 6 + block * 16;
			{
				register int byte = 8 + 2 * ( n / 45 );
				ntscmap_ch1[ n ] = sequence1 * 150 * 80 + block * 80 + byte;
				ntscmap_ch2[ n ] = sequence2 * 150 * 80 + block * 80 + byte;
				byte += ( n / 45 );
				ntscmap_2ch1[ n ] = sequence1 * 150 * 80 + block * 80 + byte;
				ntscmap_2ch2[ n ] = sequence2 * 150 * 80 + block * 80 + byte;
			}
		}
		for ( int y = 0x700; y <= 0x7ff; ++y )
			compmap[ y ] = ( y - 0x600 ) << 6;
		for ( int y = 0x600; y <= 0x6ff; ++y )
			compmap[ y ] = ( y - 0x500 ) << 5;
		for ( int y = 0x500; y <= 0x5ff; ++y )
			compmap[ y ] = ( y - 0x400 ) << 4;
		for ( int y = 0x400; y <= 0x4ff; ++y )
			compmap[ y ] = ( y - 0x300 ) << 3;
		for ( int y = 0x300; y <= 0x3ff; ++y )
			compmap[ y ] = ( y - 0x200 ) << 2;
		for ( int y = 0x200; y <= 0x2ff; ++y )
			compmap[ y ] = ( y - 0x100 ) << 1;
		for ( int y = 0x000; y <= 0x1ff; ++y )
			compmap[ y ] = y;
		for ( int y = 0x800; y <= 0xfff; ++y )
			compmap[ y ] = -1 - compmap[ 0xfff - y ];
		maps_initialized = true;
	}
#endif
	for ( int n = 0; n < 4; n++ )
		audio_buffers[ n ] = ( int16_t * ) malloc( 2 * DV_AUDIO_MAX_SAMPLES * sizeof( int16_t ) );
}


Frame::~Frame()
{
	//free( data );
	if ( tempImage )
		delete[] tempImage;

#ifdef HAVE_LIBDV
	if ( decoder )
		dv_decoder_free( decoder );
	if ( encoder )
		dv_encoder_free( encoder );
#endif

	for ( int n = 0; n < 4; n++ )
		free( audio_buffers[ n ] );
}

/** returns whether this frame was initialized successfully. */
bool Frame::IsValid() const
{
	return (decoder != NULL);
}


/** gets a subcode data packet
 
    This function returns a SSYB packet from the subcode data section.
 
    \param packNum the SSYB package id to return
    \param pack a reference to the variable where the result is stored
    \return true for success, false if no pack could be found */

bool Frame::GetSSYBPack( int packNum, Pack &pack ) const
{
	// Don't have frame data yet
	if (data == NULL) {
		return false;
	}
	
#ifdef HAVE_LIBDV
	pack.data[ 0 ] = packNum;
	dv_get_ssyb_pack( decoder, packNum, &pack.data[ 1 ] );
	return true;
#else
	/* number of DIF sequences is different for PAL and NTSC */

	int seqCount = IsPAL() ? 12 : 10;

	/* process all DIF sequences */

	for ( int i = 0; i < seqCount; ++i )
	{

		/* there are two DIF blocks in the subcode section */

		for ( int j = 0; j < 2; ++j )
		{

			/* each block has 6 packets */

			for ( int k = 0; k < 6; ++k )
			{

				/* calculate address: 150 DIF blocks per sequence, 80 bytes
				per DIF block, subcode blocks start at block 1, block and
				packet have 3 bytes header, packet is 8 bytes long
				(including header) */

				const unsigned char *s = &data[ i * 150 * 80 + 1 * 80 + j * 80 + 3 + k * 8 + 3 ];
				// printf("ssyb %d: %2.2x %2.2x %2.2x %2.2x %2.2x\n",
				// j * 6 + k, s[0], s[1], s[2], s[3], s[4]);
				if ( s[ 0 ] == packNum )
				{
					//					printf("GetSSYBPack[%x]: sequence %d, block %d, packet %d\n", packNum,i,j,k);
					pack.data[ 0 ] = s[ 0 ];
					pack.data[ 1 ] = s[ 1 ];
					pack.data[ 2 ] = s[ 2 ];
					pack.data[ 3 ] = s[ 3 ];
					pack.data[ 4 ] = s[ 4 ];
					return true;
				}
			}
		}
	}
	return false;
#endif
}


/** gets a video auxiliary data packet
 
    Every DIF block in the video auxiliary data section contains 15
    video auxiliary data packets, for a total of 45 VAUX packets. As
    the position of a VAUX packet is fixed, we could directly look it
    up, but I choose to walk through all data as with the other
    routines.
 
    \param packNum the VAUX package id to return
    \param pack a reference to the variable where the result is stored
    \return true for success, false if no pack could be found */
bool Frame::GetVAUXPack( int packNum, Pack &pack ) const
{
	// Don't have frame data yet
	if (data == NULL) {
		return false;
	}
	
#ifdef HAVE_LIBDV
	pack.data[ 0 ] = packNum;
	dv_get_vaux_pack( decoder, packNum, &pack.data[ 1 ] );
	//cerr << "VAUX: 0x"
	//<< setw(2) << setfill('0') << hex << (int) pack.data[0]
	//<< setw(2) << setfill('0') << hex << (int) pack.data[1]
	//<< setw(2) << setfill('0') << hex << (int) pack.data[2]
	//<< setw(2) << setfill('0') << hex << (int) pack.data[3]
	//<< setw(2) << setfill('0') << hex << (int) pack.data[4]
	//<< endl;
	return true;

#else
	/* number of DIF sequences is different for PAL and NTSC */

	int seqCount = IsPAL() ? 12 : 10;

	/* process all DIF sequences */

	for ( int i = 0; i < seqCount; ++i )
	{

		/* there are three DIF blocks in the VAUX section */

		for ( int j = 0; j < 3; ++j )
		{

			/* each block has 15 packets */

			for ( int k = 0; k < 15; ++k )
			{

				/* calculate address: 150 DIF blocks per sequence, 80 bytes
				per DIF block, vaux blocks start at block 3, block has 3
				bytes header, packets have no header and are 5 bytes
				long. */

				const unsigned char *s = &data[ i * 150 * 80 + 3 * 80 + j * 80 + 3 + k * 5 ];
				//printf("vaux %d: %2.2x %2.2x %2.2x %2.2x %2.2x\n",
				//	j * 15 + k, s[0],  s[1],  s[2],  s[3],  s[4]);
				if ( s[ 0 ] == packNum )
				{
					pack.data[ 0 ] = s[ 0 ];
					pack.data[ 1 ] = s[ 1 ];
					pack.data[ 2 ] = s[ 2 ];
					pack.data[ 3 ] = s[ 3 ];
					pack.data[ 4 ] = s[ 4 ];
					return true;
				}
			}
		}
	}
	return false;
#endif
}


/** gets an audio auxiliary data packet
 
    Every DIF block in the audio section contains 5 bytes audio
    auxiliary data and 72 bytes of audio data.  The function searches
    through all DIF blocks although AAUX packets are only allowed in
    certain defined DIF blocks.
 
    \param packNum the AAUX package id to return
    \param pack a reference to the variable where the result is stored
    \return true for success, false if no pack could be found */
bool Frame::GetAAUXPack( int packNum, Pack &pack ) const
{
	// Don't have frame data yet
	if (data == NULL) {
		return false;
	}
	
#ifdef HAVE_LIBDV
	bool done = false;
	switch ( packNum )
	{
	case 0x50:
		memcpy( pack.data, &decoder->audio->aaux_as, 5 );
		done = true;
		break;

	case 0x51:
		memcpy( pack.data, &decoder->audio->aaux_asc, 5 );
		done = true;
		break;

	case 0x52:
		memcpy( pack.data, &decoder->audio->aaux_as1, 5 );
		done = true;
		break;

	case 0x53:
		memcpy( pack.data, &decoder->audio->aaux_asc1, 5 );
		done = true;
		break;
	}
	if ( done )
		return true;
#endif

	/* number of DIF sequences is different for PAL and NTSC */

	int seqCount = IsPAL() ? 12 : 10;

	/* process all DIF sequences */

	for ( int i = 0; i < seqCount; ++i )
	{

		/* there are nine audio DIF blocks */
		for ( int j = 0; j < 9; ++j )
		{

			/* calculate address: 150 DIF blocks per sequence, 80 bytes
			   per DIF block, audio blocks start at every 16th beginning
			   with block 6, block has 3 bytes header, followed by one
			   packet. */

			const unsigned char *s = &data[ i * 150 * 80 + 6 * 80 + j * 16 * 80 + 3 ];
			if ( s[ 0 ] == packNum )
			{
				// printf("aaux %d: %2.2x %2.2x %2.2x %2.2x %2.2x\n",
				// j, s[0], s[1], s[2], s[3], s[4]);
				pack.data[ 0 ] = s[ 0 ];
				pack.data[ 1 ] = s[ 1 ];
				pack.data[ 2 ] = s[ 2 ];
				pack.data[ 3 ] = s[ 3 ];
				pack.data[ 4 ] = s[ 4 ];
				return true;
			}
		}
	}
	return false;
}


/** gets the timecode information of this frame
 
    Returns a string with the timecode of this frame. The timecode is
    the relative location of this frame on the tape, and is defined
    by hour, minute, second and frame (within the last second).
     
    \param timeCode the TimeCode struct
    \return true for success, false if no timecode information could be found */

bool Frame::GetTimeCode( TimeCode &timeCode ) const
{
	// Don't have frame data yet
	if (data == NULL) {
		return false;
	}
	
#ifdef HAVE_LIBDV
	int timestamp[ 4 ];

	dv_get_timestamp_int( decoder, timestamp );

	timeCode.hour = timestamp[ 0 ];
	timeCode.min = timestamp[ 1 ];
	timeCode.sec = timestamp[ 2 ];
	timeCode.frame = timestamp[ 3 ];
#else

	Pack tc;

	if ( GetSSYBPack( 0x13, tc ) == false )
		return false;

	int frame = tc.data[ 1 ];
	int sec = tc.data[ 2 ];
	int min = tc.data[ 3 ];
	int hour = tc.data[ 4 ];

	timeCode.frame = ( frame & 0xf ) + 10 * ( ( frame >> 4 ) & 0x3 );
	timeCode.sec = ( sec & 0xf ) + 10 * ( ( sec >> 4 ) & 0x7 );
	timeCode.min = ( min & 0xf ) + 10 * ( ( min >> 4 ) & 0x7 );
	timeCode.hour = ( hour & 0xf ) + 10 * ( ( hour >> 4 ) & 0x3 );
#endif

	return true;
}


/** gets the audio properties of this frame
 
    get the sampling frequency and the number of samples in this particular DV frame (which can vary)
 
    \param info the AudioInfo record
    \return true, if audio properties could be determined */

bool Frame::GetAudioInfo( AudioInfo &info ) const
{
	// Don't have frame data yet
	if (data == NULL) {
		return false;
	}
	
#ifdef HAVE_LIBDV
	info.frequency = dv_get_frequency( decoder );
	info.samples = dv_get_num_samples( decoder );
	info.frames = IsPAL() ? 50 : 60;
	info.channels = dv_get_num_channels( decoder );
	info.quantization = ( decoder->audio->aaux_as.pc4.qu == 0 ) ? 16 : 12;
	return true;
#else

	int af_size;
	int smp;
	int flag;
	Pack pack50;


	info.channels = 2;

	/* Check whether this frame has a valid AAUX source packet
	(header == 0x50). If so, get the audio samples count. If not,
	skip this audio data. */

	if ( GetAAUXPack( 0x50, pack50 ) == true )
	{

		/* get size, sampling type and the 50/60 flag. The number of
		audio samples is dependend on all of these. */

		af_size = pack50.data[ 1 ] & 0x3f;
		smp = ( pack50.data[ 4 ] >> 3 ) & 0x07;
		flag = pack50.data[ 3 ] & 0x20;

		if ( flag == 0 )
		{
			info.frames = 60;
			switch ( smp )
			{
			case 0:
				info.frequency = 48000;
				info.samples = 1580 + af_size;
				break;
			case 1:
				info.frequency = 44100;
				info.samples = 1452 + af_size;
				break;
			case 2:
				info.frequency = 32000;
				info.samples = 1053 + af_size;
				break;
			}
		}
		else
		{ // 50 frames (PAL)
			info.frames = 50;
			switch ( smp )
			{
			case 0:
				info.frequency = 48000;
				info.samples = 1896 + af_size;
				break;
			case 1:
				info.frequency = 44100;
				info.samples = 0; // I don't know
				break;
			case 2:
				info.frequency = 32000;
				info.samples = 1264 + af_size;
				break;
			}
		}
		return true;
	}
	else
	{
		return false;
	}
#endif
}

/** gets the size of the frame
 
    Depending on the type (PAL or NTSC) of the frame, the length of the frame is returned 
 
    \return the length of the frame in Bytes */

int Frame::GetFrameSize( void ) const
{
	// Don't have frame data yet
	if (data == NULL) {
		return 0;
	}
	
	return IsPAL() ? 144000 : 120000;
}


/** gets the frame rate of the video
 
    Depending on the type (PAL or NTSC) of the frame, the frame rate is returned 
 
    \return frames per second */

float Frame::GetFrameRate( void ) const
{
	// Don't have frame data yet
	if (data == NULL) {
		return 0;
	}
	
	return IsPAL() ? 25.0 : 30000.0 / 1001.0;
}


/** checks whether the frame is in PAL or NTSC format
 
    \todo function can't handle "empty" frame
    \return true for PAL frame, false for a NTSC frame
*/

bool Frame::IsPAL( void ) const
{
	// Don't have frame data yet
	if (data == NULL) {
		printf("frame.cc: IsPal: illegal state: no frames received yet\n"); 
		return false;
	}
	
	unsigned char dsf = data[ 3 ] & 0x80;
	bool pal = ( dsf == 0 ) ? false : true;

#ifdef HAVE_LIBDV
	if ( !pal )
		pal = dv_is_PAL( decoder );
#endif
	return pal;
}


/** checks whether this frame is the first in a new recording
 
    To determine this, the function looks at the recStartPoint bit in
    AAUX pack 51.
 
    \return true if this frame is the start of a new recording */

bool Frame::IsNewRecording() const
{
	// Don't have frame data yet
	if (data == NULL) {
		return false;
	}
	
#ifdef HAVE_LIBDV
	return dv_is_new_recording( decoder, data );
#else

	Pack aauxSourceControl;

	/* if we can't find the packet, we return "no new recording" */

	if ( GetAAUXPack( 0x51, aauxSourceControl ) == false )
		return false;

	unsigned char recStartPoint = aauxSourceControl.data[ 2 ] & 0x80;

	return recStartPoint == 0 ? true : false;
#endif
}


/** checks whether this frame is playing at normal speed
 
    To determine this, the function looks at the speed bit in
    AAUX pack 51.
 
    \return true if this frame is playing at normal speed 
*/

bool Frame::IsNormalSpeed() const
{
	bool normal_speed = true;

#ifdef HAVE_LIBDV
	/* don't do audio if speed is not 1 */
	return dv_is_normal_speed( decoder );
#endif

	return ( normal_speed );
}


/** check whether we have received as many bytes as expected for this frame
 
    \return true if this frames is completed, false otherwise */

bool Frame::IsComplete( void ) const
{
	return bytesInFrame >= GetFrameSize();
}


/** retrieves the audio data from the frame
 
    The DV frame contains audio data mixed in the video data blocks, 
    which can be retrieved easily using this function.
 
    The audio data consists of 16 bit, two channel audio samples (a 16 bit word for channel 1, followed by a 16 bit word
    for channel 2 etc.)
 
    \param sound a pointer to a buffer that holds the audio data
    \return the number of bytes put into the buffer, or 0 if no audio data could be retrieved */

int Frame::ExtractAudio( void *sound ) const
{
	AudioInfo info;
	
	// Don't have frame data yet
	if (data == NULL) {
		return 0;
	}

#ifdef HAVE_LIBDV

	if ( GetAudioInfo( info ) == true )
	{
		int n, i;
		int16_t* s = ( int16_t * ) sound;
		dv_decode_full_audio( decoder, data, ( int16_t ** ) audio_buffers );
		for ( n = 0; n < info.samples; ++n )
			for ( i = 0; i < info.channels; i++ )
				*s++ = audio_buffers[ i ][ n ];

	}
	else
		info.samples = 0;

#else
	/* Collect the audio samples */
	char* s = ( char * ) sound;

	GetAudioInfo( info );
	switch ( info.frequency )
	{
	case 32000:

		/* This is 4 channel audio */

		if ( IsPAL() )
		{
			short * p = ( short* ) sound;
			for ( int n = 0; n < info.samples; ++n )
			{
				register int r = ( ( unsigned char* ) data ) [ palmap_2ch1[ n ] + 1 ]; // LSB
				*p++ = compmap[ ( ( ( unsigned char* ) data ) [ palmap_2ch1[ n ] ] << 4 ) + ( r >> 4 ) ];   // MSB
				*p++ = compmap[ ( ( ( unsigned char* ) data ) [ palmap_2ch1[ n ] + 1 ] << 4 ) + ( r & 0x0f ) ];
			}


		}
		else
		{
			short* p = ( short* ) sound;
			for ( int n = 0; n < info.samples; ++n )
			{
				register int r = ( ( unsigned char* ) data ) [ ntscmap_2ch1[ n ] + 1 ]; // LSB
				*p++ = compmap[ ( ( ( unsigned char* ) data ) [ ntscmap_2ch1[ n ] ] << 4 ) + ( r >> 4 ) ];   // MSB
				*p++ = compmap[ ( ( ( unsigned char* ) data ) [ ntscmap_2ch1[ n ] + 1 ] << 4 ) + ( r & 0x0f ) ];
			}
		}
		break;

	case 44100:
	case 48000:

		/* this can be optimized significantly */

		if ( IsPAL() )
		{
			for ( int n = 0; n < info.samples; ++n )
			{
				*s++ = ( ( char* ) data ) [ palmap_ch1[ n ] + 1 ]; /* LSB */
				*s++ = ( ( char* ) data ) [ palmap_ch1[ n ] ];     /* MSB */
				*s++ = ( ( char* ) data ) [ palmap_ch2[ n ] + 1 ]; /* LSB */
				*s++ = ( ( char* ) data ) [ palmap_ch2[ n ] ];     /* MSB */
			}
		}
		else
		{
			for ( int n = 0; n < info.samples; ++n )
			{
				*s++ = ( ( char* ) data ) [ ntscmap_ch1[ n ] + 1 ]; /* LSB */
				*s++ = ( ( char* ) data ) [ ntscmap_ch1[ n ] ];     /* MSB */
				*s++ = ( ( char* ) data ) [ ntscmap_ch2[ n ] + 1 ]; /* LSB */
				*s++ = ( ( char* ) data ) [ ntscmap_ch2[ n ] ];     /* MSB */
			}
		}
		break;

		/* we can't handle any other format in the moment */

	default:
		info.samples = 0;
	}

#endif
	return info.samples * info.channels * 2;
}

#ifdef HAVE_LIBDV

void Frame::SetPreferredQuality( )
{
	switch ( PREF_DISPLAY_QUALITY )
	{
	case 5:
		dv_set_quality( decoder, DV_QUALITY_FASTEST );
		break;
	case 4:
		dv_set_quality( decoder, DV_QUALITY_AC_1 );
		break;
	case 3:
		dv_set_quality( decoder, DV_QUALITY_COLOR | DV_QUALITY_DC );
		break;
	case 2:
		dv_set_quality( decoder, DV_QUALITY_COLOR | DV_QUALITY_AC_1 );
		break;
	default:
		dv_set_quality( decoder, DV_QUALITY_BEST );
		break;
	}
}

/** retrieves the audio data from the frame
 
    The DV frame contains audio data mixed in the video data blocks, 
    which can be retrieved easily using this function.
 
    The audio data consists of 16 bit, two channel audio samples (a 16 bit word for channel 1, followed by a 16 bit word
    for channel 2 etc.)
 
    \param channels an array of buffers of audio data, one per channel, up to four channels
    \return the number of bytes put into the buffer, or 0 if no audio data could be retrieved */

int Frame::ExtractAudio( int16_t **channels ) const
{
	// Don't have frame data yet
	if (data == NULL) {
		return 0;
	}
	
	AudioInfo info;
	if ( GetAudioInfo( info ) == true )
		dv_decode_full_audio( decoder, data, channels );
	else
		info.samples = 0;

	return info.samples * info.channels * 2;
}

void Frame::ExtractHeader( void )
{
	// Don't have frame data yet
	if (data == NULL) {
		return;
	}
	
	dv_parse_header( decoder, data );
	dv_parse_packs( decoder, data );
}

void Frame::GetUpperField( void * image, int bpp )
{
	register int width = GetWidth( ) * bpp;
	register int height = GetHeight( );
	for ( register int i = 0; i < height; i += 2 )
		memcpy( ( uint8_t * ) image + width * ( i + 1 ), ( uint8_t * ) image + width * i, width );
}

void Frame::GetLowerField( void * image, int bpp )
{
	register int width = GetWidth( ) * bpp;
	register int height = GetHeight( );
	for ( register int i = 0; i < height; i += 2 )
		memcpy( ( uint8_t * ) image + width * i, ( uint8_t * ) image + width * ( i + 1 ), width );
}

/* Linear Blend filter - C version contributed by Rogerio Brito.
   This algorithm has the same interface as the other functions.

   The destination "screen" (pdst) is constructed from the source
   screen (psrc[0]) line by line.

   The i-th line of the destination screen is the average of 3 lines
   from the source screen: the (i-1)-th, i-th and (i+1)-th lines, with
   the i-th line having weight 2 in the computation.

   Remarks:
   * each line on pdst doesn't depend on previous lines;
   * due to the way the algorithm is defined, the first & last lines of the
     screen aren't deinterlaced.

*/
void Frame::Deinterlace( uint8_t *pdst, uint8_t *psrc, int stride, int height )
{
	register int x, y;
	register uint8_t *l0, *l1, *l2, *l3;

	l0 = pdst;		/* target line */
	l1 = psrc;		/* 1st source line */
	l2 = l1 + stride;	/* 2nd source line = line that follows l1 */
	l3 = l2 + stride;	/* 3rd source line = line that follows l2 */

	/* Copy the first line */
	memcpy( l0, l1, stride );
	l0 += stride;

	for (y = 1; y < height-1; ++y)
	{
		/* computes avg of: l1 + 2*l2 + l3 */
		for ( x = 0; x < stride; ++x )
			l0[x] = ( l1[ x ] + ( l2[ x ] << 1 ) + l3[ x ] ) >> 2;

		/* updates the line pointers */
		l1 = l2;
		l2 = l3;
		l3 += stride;
		l0 += stride;
	}

	/* Copy the last line */
	memcpy( l0, l1, stride );
}


int Frame::ExtractRGB( void * rgb )
{
	// Don't have frame data yet
	if (data == NULL) {
		return 0;
	}
	
	unsigned char *pixels[ 3 ];
	int pitches[ 3 ];

	pixels[ 0 ] = ( unsigned char* ) rgb;
	pixels[ 1 ] = NULL;


	pixels[ 2 ] = NULL;

	pitches[ 0 ] = 720 * 3;
	pitches[ 1 ] = 0;
	pitches[ 2 ] = 0;

	dv_decode_full_frame( decoder, data, e_dv_color_rgb, pixels, pitches );
	return 0;
}

int Frame::ExtractYUV( void *yuv )
{
	// Don't have frame data yet
	if (data == NULL) {
		return 0;
	}
	
	unsigned char *pixels[ 3 ];
	int pitches[ 3 ];

	pixels[ 0 ] = ( unsigned char* ) yuv;
	pitches[ 0 ] = decoder->width * 2;

	dv_decode_full_frame( decoder, data, e_dv_color_yuv, pixels, pitches );
	return 0;
}

int Frame::ExtractYUV420( uint8_t *yuv, uint8_t *output[ 3 ] )
{
	// Don't have frame data yet
	if (data == NULL) {
		return 0;
	}
	
	unsigned char *pixels[ 3 ];
	int pitches[ 3 ];
	int width = GetWidth(), height = GetHeight();

	pixels[ 0 ] = ( unsigned char* ) yuv;
	pitches[ 0 ] = decoder->width * 2;

	dv_decode_full_frame( decoder, data, e_dv_color_yuv, pixels, pitches );

	int w2 = width / 2;
	uint8_t *y = output[ 0 ];
	uint8_t *cb = output[ 1 ];
	uint8_t *cr = output[ 2 ];
	uint8_t *p = yuv;

	for ( int i = 0; i < height; i += 2 )
	{
		/* process two scanlines (one from each field, interleaved) */
		for ( int j = 0; j < w2; j++ )
		{
			/* packed YUV 422 is: Y[i] U[i] Y[i+1] V[i] */
			*( y++ ) = *( p++ );
			*( cb++ ) = *( p++ );
			*( y++ ) = *( p++ );
			*( cr++ ) = *( p++ );
		}
		/* process next two scanlines (one from each field, interleaved) */
		for ( int j = 0; j < w2; j++ )
		{
			/* skip every second line for U and V */
			*( y++ ) = *( p++ );
			p++;
			*( y++ ) = *( p++ );
			p++;
		}
	}
	return 0;
}

/** Get the frame aspect ratio.
 
	Indicates whether frame aspect ration is normal (4:3) or wide (16:9).
 
    \return true if the frame is wide (16:9), false if unknown or normal.
*/
bool Frame::IsWide( void ) const
{
	return dv_format_wide( decoder ) > 0;
}


/** Get the frame image width.
 
    \return the width in pixels.
*/
int Frame::GetWidth() const
{
	return 720;
}


/** Get the frame image height.
 
    \return the height in pixels.
*/
int Frame::GetHeight() const
{
	return IsPAL() ? 576 : 480;
}


/** Set the RecordingDate of the frame.
 
    This updates the calendar date and time and the timecode.
    However, timecode is derived from the time in the datetime
    parameter and frame number. Use SetTimeCode for more control
    over timecode.
 
    \param datetime A simple time value containing the
           RecordingDate and time information. The time in this
           structure is automatically incremented by one second
           depending on the frame parameter and updatded.
    \param frame A zero-based running frame sequence/serial number.
           This is used both in the timecode as well as a timestamp on
           dif block headers.
*/
void Frame::SetRecordingDate( time_t *datetime, int frame )
{
	// Don't have frame data yet
	if (data == NULL) {
		return;
	}
	
	dv_encode_metadata( data, IsPAL(), IsWide(), datetime, frame );
}

/** Set the TimeCode of the frame.
 
    This function takes a zero-based frame counter and automatically
    derives the timecode.
 
    \param frame The frame counter.
*/
void Frame::SetTimeCode( int frame )
{
	// Don't have frame data yet
	if (data == NULL) {
		return;
	}
	
	dv_encode_timecode( data, IsPAL(), frame );
}

bool Frame::CreateEncoder( bool isPAL, bool isWide )
{
	if ( encoder == NULL )
	{
		if ( isPAL )
			encoder = dv_encoder_new( FALSE,
			                          FALSE,	// PAL always clamps luma
			                          FALSE );	// PAL always clamps chroma
		else
			encoder = dv_encoder_new( FALSE,
			                          SHOULD_CLAMP_LUMA,
			                          SHOULD_CLAMP_CHROMA );
		if ( encoder )
		{
			encoder->isPAL = isPAL;
			encoder->is16x9 = isWide;
			encoder->vlc_encode_passes = 3;
			encoder->static_qno = 0;
			encoder->force_dct = DV_DCT_AUTO;

			tempImage = new uint8_t[ FRAME_MAX_WIDTH * FRAME_MAX_HEIGHT * 4 ];
		}
	}
	return ( encoder != NULL );
}

bool Frame::EncodeAudio( AudioInfo &info, int16_t **channels )
{
	// Don't have frame data yet
	if (data == NULL) {
		return false;
	}
	
	int result = -1;

	if ( CreateEncoder( IsPAL(), IsWide() ) )
	{
		encoder->samples_this_frame = info.samples;
		result = dv_encode_full_audio( encoder, channels, info.channels, info.frequency, data );
	}
	return ( result != -1 );
}

int Frame::CalculateNumberSamples( int frequency, int iteration )
{
	int samples = 0;

	if ( CreateEncoder( IsPAL(), IsWide() ) )
		samples = dv_calculate_samples( encoder, frequency, iteration );
	
	return samples;
}

void Frame::EncodeRGB( uint8_t *rgb )
{
	// Don't have frame data yet
	if (data == NULL) {
		return;
	}
	
	if ( CreateEncoder( IsPAL(), IsWide() ) )
	{
		if ( USE_TWO_PASS_DV_ENCODER )
		{
			dv_encode_full_frame( encoder, &rgb, e_dv_color_rgb, data );
			decoder->quality = DV_QUALITY_BEST;
			ExtractHeader( );
			ExtractRGB( tempImage );
			int size = GetWidth( ) * GetHeight( ) * 3;
			for ( int i = 0; i < size; i ++ )
				rgb[ i ] = std::min( std::max( ( int ) rgb[ i ] - ( ( int ) tempImage[ i ] - ( int ) rgb[ i ] ), 0 ), 255 );
			dv_encode_full_frame( encoder, &rgb, e_dv_color_rgb, data );
			SetPreferredQuality();
			ExtractHeader();
		}
		else
		{
			dv_encode_full_frame( encoder, &rgb, e_dv_color_rgb, data );
		}
	}
}

#endif
