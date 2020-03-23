// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    sound.c

    Core sound functions and definitions.

***************************************************************************/

#include "emu.h"
#include "speaker.h"
#include "emuopts.h"
#include "osdepend.h"
#include "config.h"
#include "wavwrite.h"



//**************************************************************************
//  DEBUGGING
//**************************************************************************

#define VERBOSE         (0)

#define VPRINTF(x)      do { if (VERBOSE) osd_printf_debug x; } while (0)



//**************************************************************************
//  CONSTANTS
//**************************************************************************



//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

const attotime sound_manager::STREAMS_UPDATE_ATTOTIME = attotime::from_hz(STREAMS_UPDATE_FREQUENCY);



//**************************************************************************
//  INITIALIZATION
//**************************************************************************

//-------------------------------------------------
//  sound_stream - constructor
//-------------------------------------------------

sound_stream::sound_stream(device_t &device, int inputs, int outputs, int sample_rate,  stream_update_delegate callback)
	: m_device(device),
		m_next(nullptr),
		m_sample_rate(sample_rate),
		m_new_sample_rate(0),
		m_attoseconds_per_sample(0),
		m_max_samples_per_update(0),
		m_input(inputs),
		m_input_array(inputs),
		m_resample_bufalloc(0),
		m_output(outputs),
		m_output_array(outputs),
		m_output_bufalloc(0),
		m_output_sampindex(0),
		m_output_update_sampindex(0),
		m_output_base_sampindex(0),
		m_callback(std::move(callback))
{
	// get the device's sound interface
	device_sound_interface *sound;
	if (!device.interface(sound))
		throw emu_fatalerror("Attempted to create a sound_stream with a non-sound device");

	if(m_callback.isnull())
		m_callback = stream_update_delegate(&device_sound_interface::sound_stream_update,(device_sound_interface *)sound);

	// create a unique tag for saving
	std::string state_tag = string_format("%d", m_device.machine().sound().m_stream_list.size());
	m_device.machine().save().save_item(&m_device, "stream", state_tag.c_str(), 0, NAME(m_sample_rate));
	m_device.machine().save().register_postload(save_prepost_delegate(FUNC(sound_stream::postload), this));

	// save the gain of each input and output
	for (unsigned int inputnum = 0; inputnum < m_input.size(); inputnum++)
	{
		m_device.machine().save().save_item(&m_device, "stream", state_tag.c_str(), inputnum, NAME(m_input[inputnum].m_gain));
		m_device.machine().save().save_item(&m_device, "stream", state_tag.c_str(), inputnum, NAME(m_input[inputnum].m_user_gain));
	}
	for (unsigned int outputnum = 0; outputnum < m_output.size(); outputnum++)
	{
		m_output[outputnum].m_stream = this;
		m_device.machine().save().save_item(&m_device, "stream", state_tag.c_str(), outputnum, NAME(m_output[outputnum].m_gain));
	}

	// Mark synchronous streams as such
	m_synchronous = m_sample_rate == STREAM_SYNC;
	if (m_synchronous)
	{
		m_sample_rate = 0;
		m_sync_timer = m_device.machine().scheduler().timer_alloc(timer_expired_delegate(FUNC(sound_stream::sync_update), this));
	}
	else
		m_sync_timer = nullptr;

	// force an update to the sample rates; this will cause everything to be recomputed
	// and will generate the initial resample buffers for our inputs
	recompute_sample_rate_data();

	// set up the initial output buffer positions now that we have data
	m_output_base_sampindex = -m_max_samples_per_update;
}


//-------------------------------------------------
//  sample_time - return the emulation time of the
//  next sample to be generated on the stream
//-------------------------------------------------

attotime sound_stream::sample_time() const
{
	return attotime(m_device.machine().sound().last_update().seconds(), 0) + attotime(0, m_output_sampindex * m_attoseconds_per_sample);
}


//-------------------------------------------------
//  user_gain - return the user-controllable gain
//  on a given stream's input
//-------------------------------------------------

float sound_stream::user_gain(int inputnum) const
{
	assert(inputnum >= 0 && inputnum < m_input.size());
	return float(m_input[inputnum].m_user_gain) / 256.0f;
}


//-------------------------------------------------
//  input_gain - return the input gain on a
//  given stream's input
//-------------------------------------------------

float sound_stream::input_gain(int inputnum) const
{
	assert(inputnum >= 0 && inputnum < m_input.size());
	return float(m_input[inputnum].m_gain) / 256.0f;
}


//-------------------------------------------------
//  output_gain - return the output gain on a
//  given stream's output
//-------------------------------------------------

float sound_stream::output_gain(int outputnum) const
{
	assert(outputnum >= 0 && outputnum < m_output.size());
	return float(m_output[outputnum].m_gain) / 256.0f;
}


//-------------------------------------------------
//  input_name - return the original input gain
//  on a given stream's input
//-------------------------------------------------

std::string sound_stream::input_name(int inputnum) const
{
	std::ostringstream str;

	// start with our device name and tag
	assert(inputnum >= 0 && inputnum < m_input.size());
	util::stream_format(str, "%s '%s': ", m_device.name(), m_device.tag());

	// if we have a source, indicate where the sound comes from by device name and tag
	if (m_input[inputnum].m_source != nullptr && m_input[inputnum].m_source->m_stream != nullptr)
	{
		device_t &source = m_input[inputnum].m_source->m_stream->device();
		util::stream_format(str, "%s '%s'", source.name(), source.tag());

		// get the sound interface; if there is more than 1 output we need to figure out which one
		device_sound_interface *sound;
		if (source.interface(sound) && sound->outputs() > 1)
		{
			// iterate over outputs until we find the stream that matches our source
			// then look for a match on the output number
			sound_stream *outstream;
			int streamoutputnum;
			for (int outputnum = 0; (outstream = sound->output_to_stream_output(outputnum, streamoutputnum)) != nullptr; outputnum++)
				if (outstream == m_input[inputnum].m_source->m_stream && m_input[inputnum].m_source == &outstream->m_output[streamoutputnum])
				{
					util::stream_format(str, " Ch.%d", outputnum);
					break;
				}
		}
	}
	return str.str();
}


//-------------------------------------------------
//  input_source_device - return the device
//  attached as a given input's source
//-------------------------------------------------

device_t *sound_stream::input_source_device(int inputnum) const
{
	assert(inputnum >= 0 && inputnum < m_input.size());
	return (m_input[inputnum].m_source != nullptr) ? &m_input[inputnum].m_source->m_stream->device() : nullptr;
}


//-------------------------------------------------
//  input_source_device - return the output number
//  attached as a given input's source
//-------------------------------------------------

int sound_stream::input_source_outputnum(int inputnum) const
{
	assert(inputnum >= 0 && inputnum < m_input.size());
	return (m_input[inputnum].m_source != nullptr) ? (m_input[inputnum].m_source - &m_input[inputnum].m_source->m_stream->m_output[0]) : -1;
}


//-------------------------------------------------
//  set_input - configure a stream's input
//-------------------------------------------------

void sound_stream::set_input(int index, sound_stream *input_stream, int output_index, float gain)
{
	VPRINTF(("stream_set_input(%p, '%s', %d, %p, %d, %f)\n", (void *)this, m_device.tag(),
			index, (void *)input_stream, output_index, (double) gain));

	// make sure it's a valid input
	if (index >= m_input.size())
		fatalerror("stream_set_input attempted to configure non-existant input %d (%d max)\n", index, int(m_input.size()));

	// make sure it's a valid output
	if (input_stream != nullptr && output_index >= input_stream->m_output.size())
		fatalerror("stream_set_input attempted to use a non-existant output %d (%d max)\n", output_index, int(m_output.size()));

	// if this input is already wired, update the dependent info
	stream_input &input = m_input[index];
	if (input.m_source != nullptr)
		input.m_source->m_dependents--;

	// wire it up
	input.m_source = (input_stream != nullptr) ? &input_stream->m_output[output_index] : nullptr;
	input.m_gain = int(0x100 * gain);
	input.m_user_gain = 0x100;

	// update the dependent info
	if (input.m_source != nullptr)
		input.m_source->m_dependents++;

	// update sample rates now that we know the input
	recompute_sample_rate_data();
}


//-------------------------------------------------
//  update - force a stream to update to
//  the current emulated time
//-------------------------------------------------

void sound_stream::update()
{
	// determine the number of samples since the start of this second
	attotime time = m_device.machine().time();
	s32 update_sampindex = s32(time.attoseconds() / m_attoseconds_per_sample);

	// if we're ahead of the last update, then adjust upwards
	attotime last_update = m_device.machine().sound().last_update();
	if (time.seconds() > last_update.seconds())
	{
		assert(time.seconds() == last_update.seconds() + 1);
		update_sampindex += m_sample_rate;
	}

	// if we're behind the last update, then adjust downwards
	if (time.seconds() < last_update.seconds())
	{
		assert(time.seconds() == last_update.seconds() - 1);
		update_sampindex -= m_sample_rate;
	}

	// generate samples to get us up to the appropriate time
	g_profiler.start(PROFILER_SOUND);
	assert(m_output_sampindex - m_output_base_sampindex >= 0);
	assert(update_sampindex - m_output_base_sampindex <= m_output_bufalloc);
	generate_samples(update_sampindex - m_output_sampindex);
	g_profiler.stop();

	// remember this info for next time
	m_output_sampindex = update_sampindex;
}


void sound_stream::sync_update(void *, s32)
{
	update();
	attotime time = m_device.machine().time();
	attoseconds_t next_edge = m_attoseconds_per_sample - (time.attoseconds() % m_attoseconds_per_sample);
	m_sync_timer->adjust(attotime(0, next_edge));
}


//-------------------------------------------------
//  output_since_last_update - return a pointer to
//  the output buffer and the number of samples
//  since the last global update
//-------------------------------------------------

const stream_sample_t *sound_stream::output_since_last_update(int outputnum, int &numsamples)
{
	// force an update on the stream
	update();

	// compute the number of samples and a pointer to the output buffer
	numsamples = m_output_sampindex - m_output_update_sampindex;
	return &m_output[outputnum].m_buffer[m_output_update_sampindex - m_output_base_sampindex];
}


//-------------------------------------------------
//  set_sample_rate - set the sample rate on a
//  given stream
//-------------------------------------------------

void sound_stream::set_sample_rate(int new_rate)
{
	// we will update this on the next global update
	if (new_rate != sample_rate())
		m_new_sample_rate = new_rate;
}


//-------------------------------------------------
//  set_user_gain - set the user-controllable gain
//  on a given stream's input
//-------------------------------------------------

void sound_stream::set_user_gain(int inputnum, float gain)
{
	update();
	assert(inputnum >= 0 && inputnum < m_input.size());
	m_input[inputnum].m_user_gain = int(0x100 * gain);
}


//-------------------------------------------------
//  set_input_gain - set the input gain on a
//  given stream's input
//-------------------------------------------------

void sound_stream::set_input_gain(int inputnum, float gain)
{
	update();
	assert(inputnum >= 0 && inputnum < m_input.size());
	m_input[inputnum].m_gain = int(0x100 * gain);
}


//-------------------------------------------------
//  set_output_gain - set the output gain on a
//  given stream's output
//-------------------------------------------------

void sound_stream::set_output_gain(int outputnum, float gain)
{
	update();
	assert(outputnum >= 0 && outputnum < m_output.size());
	m_output[outputnum].m_gain = int(0x100 * gain);
}


//-------------------------------------------------
//  update_with_accounting - do a regular update,
//  but also do periodic accounting
//-------------------------------------------------

void sound_stream::update_with_accounting(bool second_tick)
{
	// do the normal update
	update();

	// if we've ticked over another second, adjust all the counters that are relative to
	// the current second
	s32 output_bufindex = m_output_sampindex - m_output_base_sampindex;
	if (second_tick)
	{
		m_output_sampindex -= m_sample_rate;
		m_output_base_sampindex -= m_sample_rate;
	}

	// note our current output sample
	m_output_update_sampindex = m_output_sampindex;

	// if we don't have enough output buffer space to hold two updates' worth of samples,
	// we need to shuffle things down
	if (m_output_bufalloc - output_bufindex < 2 * m_max_samples_per_update)
	{
		s32 samples_to_lose = output_bufindex - m_max_samples_per_update;
		if (samples_to_lose > 0)
		{
			// if we have samples to move, do so for each output
			if (output_bufindex > 0)
				for (auto & output : m_output)
				{
					memmove(&output.m_buffer[0], &output.m_buffer[samples_to_lose], sizeof(output.m_buffer[0]) * (output_bufindex - samples_to_lose));
				}

			// update the base position
			m_output_base_sampindex += samples_to_lose;
		}
	}
}


//-------------------------------------------------
//  apply_sample_rate_changes - if there is a
//  pending sample rate change, apply it now
//-------------------------------------------------

void sound_stream::apply_sample_rate_changes()
{
	// skip if nothing to do
	if (m_new_sample_rate == 0)
		return;

	// update to the new rate and remember the old rate
	u32 old_rate = m_sample_rate;
	m_sample_rate = m_new_sample_rate;
	m_new_sample_rate = 0;

	// recompute all the data
	recompute_sample_rate_data();

	// reset our sample indexes to the current time
	m_output_sampindex = s64(m_output_sampindex) * s64(m_sample_rate) / old_rate;
	m_output_update_sampindex = s64(m_output_update_sampindex) * s64(m_sample_rate) / old_rate;
	m_output_base_sampindex = m_output_sampindex - m_max_samples_per_update;

	// clear out the buffer
	for (auto & elem : m_output)
		memset(&elem.m_buffer[0], 0, m_max_samples_per_update * sizeof(elem.m_buffer[0]));
}


//-------------------------------------------------
//  recompute_sample_rate_data - recompute sample
//  rate data, and all streams that are affected
//  by this stream
//-------------------------------------------------

void sound_stream::recompute_sample_rate_data()
{
	if (m_synchronous)
	{
		m_sample_rate = 0;
		// When synchronous, pick the sample rate for the inputs, if any
		for (auto & input : m_input)
		{
			if (input.m_source != nullptr)
			{
				if (!m_sample_rate)
					m_sample_rate = input.m_source->m_stream->m_sample_rate;
				else if (m_sample_rate != input.m_source->m_stream->m_sample_rate)
					throw emu_fatalerror("Incompatible sample rates as input of a synchronous stream: %d and %d\n", m_sample_rate, input.m_source->m_stream->m_sample_rate);
			}
		}
		if (!m_sample_rate)
			m_sample_rate = 1000;
	}


	// recompute the timing parameters
	attoseconds_t update_attoseconds = m_device.machine().sound().update_attoseconds();
	m_attoseconds_per_sample = ATTOSECONDS_PER_SECOND / m_sample_rate;
	m_max_samples_per_update = (update_attoseconds + m_attoseconds_per_sample - 1) / m_attoseconds_per_sample;

	// update resample and output buffer sizes
	allocate_resample_buffers();
	allocate_output_buffers();

	// iterate over each input
	for (auto & input : m_input)
	{
		// if we have a source, see if its sample rate changed

		if (input.m_source != nullptr)
		{
			// okay, we have a new sample rate; recompute the latency to be the maximum
			// sample period between us and our input
			attoseconds_t new_attosecs_per_sample = ATTOSECONDS_PER_SECOND / input.m_source->m_stream->m_sample_rate;
			attoseconds_t latency = std::max(new_attosecs_per_sample, m_attoseconds_per_sample);

			// if the input stream's sample rate is lower, we will use linear interpolation
			// this requires an extra sample from the source
			if (input.m_source->m_stream->m_sample_rate < m_sample_rate)
				latency += new_attosecs_per_sample;

			// if our sample rates match exactly, we don't need any latency
			else if (input.m_source->m_stream->m_sample_rate == m_sample_rate)
				latency = 0;

			// we generally don't want to tweak the latency, so we just keep the greatest
			// one we've computed thus far
			input.m_latency_attoseconds = std::max(input.m_latency_attoseconds, latency);
			assert(input.m_latency_attoseconds < update_attoseconds);
		}
	}

	// If synchronous, prime the timer
	if (m_synchronous)
	{
		attotime time = m_device.machine().time();
		attoseconds_t next_edge = m_attoseconds_per_sample - (time.attoseconds() % m_attoseconds_per_sample);
		m_sync_timer->adjust(attotime(0, next_edge));
	}
}


//-------------------------------------------------
//  allocate_resample_buffers - recompute the
//  resample buffer sizes and expand if necessary
//-------------------------------------------------

void sound_stream::allocate_resample_buffers()
{
	// compute the target number of samples
	s32 bufsize = 2 * m_max_samples_per_update;

	// if we don't have enough room, allocate more
	if (m_resample_bufalloc < bufsize)
	{
		// this becomes the new allocation size
		m_resample_bufalloc = bufsize;

		// iterate over outputs and realloc their buffers
		for (auto & elem : m_input) {
			unsigned int old_size = elem.m_resample.size();
			elem.m_resample.resize(m_resample_bufalloc);
			memset(&elem.m_resample[old_size], 0, (m_resample_bufalloc - old_size)*sizeof(elem.m_resample[0]));
		}
	}
}


//-------------------------------------------------
//  allocate_output_buffers - recompute the
//  output buffer sizes and expand if necessary
//-------------------------------------------------

void sound_stream::allocate_output_buffers()
{
	// if we don't have enough room, allocate more
	s32 bufsize = OUTPUT_BUFFER_UPDATES * m_max_samples_per_update;
	if (m_output_bufalloc < bufsize)
	{
		// this becomes the new allocation size
		m_output_bufalloc = bufsize;

		// iterate over outputs and realloc their buffers
		for (auto & elem : m_output) {
			unsigned int old_size = elem.m_buffer.size();
			elem.m_buffer.resize(m_output_bufalloc);
			memset(&elem.m_buffer[old_size], 0, (m_output_bufalloc - old_size)*sizeof(elem.m_buffer[0]));
		}
	}
}


//-------------------------------------------------
//  postload - save/restore callback
//-------------------------------------------------

void sound_stream::postload()
{
	// recompute the same rate information
	recompute_sample_rate_data();

	// make sure our output buffers are fully cleared
	for (auto & elem : m_output)
		memset(&elem.m_buffer[0], 0, m_output_bufalloc * sizeof(elem.m_buffer[0]));

	// recompute the sample indexes to make sense
	m_output_sampindex = m_device.machine().sound().last_update().attoseconds() / m_attoseconds_per_sample;
	m_output_update_sampindex = m_output_sampindex;
	m_output_base_sampindex = m_output_sampindex - m_max_samples_per_update;
}


//-------------------------------------------------
//  generate_samples - generate the requested
//  number of samples for a stream, making sure
//  all inputs have the appropriate number of
//  samples generated
//-------------------------------------------------

void sound_stream::generate_samples(int samples)
{
	stream_sample_t **inputs = nullptr;
	stream_sample_t **outputs = nullptr;
	// if we're already there, skip it
	if (samples <= 0)
		return;

	VPRINTF(("generate_samples(%p, %d)\n", (void *) this, samples));

	// ensure all inputs are up to date and generate resampled data
	for (unsigned int inputnum = 0; inputnum < m_input.size(); inputnum++)
	{
		// update the stream to the current time
		stream_input &input = m_input[inputnum];
		if (input.m_source != nullptr)
			input.m_source->m_stream->update();

		// generate the resampled data
		m_input_array[inputnum] = generate_resampled_data(input, samples);
	}

	if (!m_input.empty())
	{
		inputs = &m_input_array[0];
	}

	// loop over all outputs and compute the output pointer
	for (unsigned int outputnum = 0; outputnum < m_output.size(); outputnum++)
	{
		stream_output &output = m_output[outputnum];
		m_output_array[outputnum] = &output.m_buffer[m_output_sampindex - m_output_base_sampindex];
	}

	if (!m_output.empty())
	{
		outputs = &m_output_array[0];
	}

	// run the callback
	VPRINTF(("  callback(%p, %d)\n", (void *)this, samples));
	m_callback(*this, inputs, outputs, samples);
	VPRINTF(("  callback done\n"));
}


//-------------------------------------------------
//  generate_resampled_data - generate the
//  resample buffer for a given input
//-------------------------------------------------

stream_sample_t *sound_stream::generate_resampled_data(stream_input &input, u32 numsamples)
{
	// if we don't have an output to pull data from, generate silence
	stream_sample_t *dest = &input.m_resample[0];
	if (input.m_source == nullptr)
	{
		memset(dest, 0, numsamples * sizeof(*dest));
		return &input.m_resample[0];
	}

	// grab data from the output
	stream_output &output = *input.m_source;
	sound_stream &input_stream = *output.m_stream;
	s64 gain = (input.m_gain * input.m_user_gain * output.m_gain) >> 16;

	// determine the time at which the current sample begins, accounting for the
	// latency we calculated between the input and output streams
	attoseconds_t basetime = m_output_sampindex * m_attoseconds_per_sample - input.m_latency_attoseconds;

	// now convert that time into a sample in the input stream
	s32 basesample;
	if (basetime >= 0)
		basesample = basetime / input_stream.m_attoseconds_per_sample;
	else
		basesample = -(-basetime / input_stream.m_attoseconds_per_sample) - 1;

	// compute a source pointer to the first sample
	assert(basesample >= input_stream.m_output_base_sampindex);
	stream_sample_t *source = &output.m_buffer[basesample - input_stream.m_output_base_sampindex];

	// determine the current fraction of a sample, expressed as a fraction of FRAC_ONE
	// (Note: this formula is valid as long as input_stream.m_attoseconds_per_sample significantly exceeds FRAC_ONE > attoseconds = 4.2E-12 s)
	u32 basefrac = (basetime - basesample * input_stream.m_attoseconds_per_sample) / ((input_stream.m_attoseconds_per_sample + FRAC_ONE - 1) >> FRAC_BITS);
	assert(basefrac < FRAC_ONE);

	// compute the stepping fraction
	u32 step = (u64(input_stream.m_sample_rate) << FRAC_BITS) / m_sample_rate;

	// if we have equal sample rates, we just need to copy
	if (step == FRAC_ONE)
	{
		while (numsamples--)
		{
			// compute the sample
			s64 sample = *source++;
			*dest++ = (sample * gain) >> 8;
		}
	}

	// input is undersampled: point sample except where our sample period covers a boundary
	else if (step < FRAC_ONE)
	{
		while (numsamples != 0)
		{
			// fill in with point samples until we hit a boundary
			int nextfrac;
			while ((nextfrac = basefrac + step) < FRAC_ONE && numsamples--)
			{
				*dest++ = (source[0] * gain) >> 8;
				basefrac = nextfrac;
			}

			// if we're done, we're done
			if (s32(numsamples--) < 0)
				break;

			// compute starting and ending fractional positions
			int startfrac = basefrac >> (FRAC_BITS - 12);
			int endfrac = nextfrac >> (FRAC_BITS - 12);

			// blend between the two samples accordingly
			s64 sample = (s64(source[0]) * (0x1000 - startfrac) + s64(source[1]) * (endfrac - 0x1000)) / (endfrac - startfrac);
			*dest++ = (sample * gain) >> 8;

			// advance
			basefrac = nextfrac & FRAC_MASK;
			source++;
		}
	}

	// input is oversampled: sum the energy
	else
	{
		// use 8 bits to allow some extra headroom
		int smallstep = step >> (FRAC_BITS - 8);
		while (numsamples--)
		{
			s64 remainder = smallstep;
			int tpos = 0;

			// compute the sample
			s64 scale = (FRAC_ONE - basefrac) >> (FRAC_BITS - 8);
			s64 sample = s64(source[tpos++]) * scale;
			remainder -= scale;
			while (remainder > 0x100)
			{
				sample += s64(source[tpos++]) * s64(0x100);
				remainder -= 0x100;
			}
			sample += s64(source[tpos]) * remainder;
			sample /= smallstep;

			*dest++ = (sample * gain) >> 8;

			// advance
			basefrac += step;
			source += basefrac >> FRAC_BITS;
			basefrac &= FRAC_MASK;
		}
	}

	return &input.m_resample[0];
}



//**************************************************************************
//  STREAM INPUT
//**************************************************************************

//-------------------------------------------------
//  stream_input - constructor
//-------------------------------------------------

sound_stream::stream_input::stream_input()
	: m_source(nullptr),
		m_latency_attoseconds(0),
		m_gain(0x100),
		m_user_gain(0x100)
{
}



//**************************************************************************
//  STREAM OUTPUT
//**************************************************************************

//-------------------------------------------------
//  stream_output - constructor
//-------------------------------------------------

sound_stream::stream_output::stream_output()
	: m_stream(nullptr),
		m_dependents(0),
		m_gain(0x100)
{
}



//**************************************************************************
//  SOUND MANAGER
//**************************************************************************

//-------------------------------------------------
//  sound_manager - constructor
//-------------------------------------------------

sound_manager::sound_manager(running_machine &machine)
	: m_machine(machine),
		m_update_timer(nullptr),
		m_finalmix_leftover(0),
		m_finalmix(machine.sample_rate()),
		m_leftmix(machine.sample_rate()),
		m_rightmix(machine.sample_rate()),
		m_muted(0),
		m_attenuation(0),
		m_nosound_mode(machine.osd().no_sound()),
		m_wavfile(nullptr),
		m_update_attoseconds(STREAMS_UPDATE_ATTOTIME.attoseconds()),
		m_last_update(attotime::zero)
{
	// get filename for WAV file or AVI file if specified
	const char *wavfile = machine.options().wav_write();
	const char *avifile = machine.options().avi_write();

	// handle -nosound and lower sample rate if not recording WAV or AVI
	if (m_nosound_mode && wavfile[0] == 0 && avifile[0] == 0)
		machine.m_sample_rate = 11025;

	// count the mixers
#if VERBOSE
	mixer_interface_iterator iter(machine.root_device());
	VPRINTF(("total mixers = %d\n", iter.count()));
#endif

	// register callbacks
	machine.configuration().config_register("mixer", config_load_delegate(&sound_manager::config_load, this), config_save_delegate(&sound_manager::config_save, this));
	machine.add_notifier(MACHINE_NOTIFY_PAUSE, machine_notify_delegate(&sound_manager::pause, this));
	machine.add_notifier(MACHINE_NOTIFY_RESUME, machine_notify_delegate(&sound_manager::resume, this));
	machine.add_notifier(MACHINE_NOTIFY_RESET, machine_notify_delegate(&sound_manager::reset, this));
	machine.add_notifier(MACHINE_NOTIFY_EXIT, machine_notify_delegate(&sound_manager::stop_recording, this));

	// register global states
	machine.save().save_item(NAME(m_last_update));

	// set the starting attenuation
	set_attenuation(machine.options().volume());

	// start the periodic update flushing timer
	m_update_timer = machine.scheduler().timer_alloc(timer_expired_delegate(FUNC(sound_manager::update), this));
	m_update_timer->adjust(STREAMS_UPDATE_ATTOTIME, 0, STREAMS_UPDATE_ATTOTIME);
}


//-------------------------------------------------
//  sound_manager - destructor
//-------------------------------------------------

sound_manager::~sound_manager()
{
}


//-------------------------------------------------
//  start_recording - begin audio recording
//-------------------------------------------------

void sound_manager::start_recording()
{
	// open the output WAV file if specified
	const char *wavfile = machine().options().wav_write();
	if (wavfile[0] != 0 && m_wavfile == nullptr)
		m_wavfile = wav_open(wavfile, machine().sample_rate(), 2);
}


//-------------------------------------------------
//  stop_recording - end audio recording
//-------------------------------------------------

void sound_manager::stop_recording()
{
	// close any open WAV file
	if (m_wavfile != nullptr)
		wav_close(m_wavfile);
	m_wavfile = nullptr;
}


//-------------------------------------------------
//  stream_alloc - allocate a new stream
//-------------------------------------------------

sound_stream *sound_manager::stream_alloc(device_t &device, int inputs, int outputs, int sample_rate, stream_update_delegate callback)
{
	m_stream_list.push_back(std::make_unique<sound_stream>(device, inputs, outputs, sample_rate, callback));
	return m_stream_list.back().get();
}


//-------------------------------------------------
//  set_attenuation - set the global volume
//-------------------------------------------------

void sound_manager::set_attenuation(int attenuation)
{
	m_attenuation = attenuation;
	machine().osd().set_mastervolume(m_muted ? -32 : m_attenuation);
}


//-------------------------------------------------
//  indexed_mixer_input - return the mixer
//  device and input index of the global mixer
//  input
//-------------------------------------------------

bool sound_manager::indexed_mixer_input(int index, mixer_input &info) const
{
	// scan through the mixers until we find the indexed input
	for (device_mixer_interface &mixer : mixer_interface_iterator(machine().root_device()))
	{
		if (index < mixer.inputs())
		{
			info.mixer = &mixer;
			info.stream = mixer.input_to_stream_input(index, info.inputnum);
			assert(info.stream != nullptr);
			return true;
		}
		index -= mixer.inputs();
	}

	// didn't locate
	info.mixer = nullptr;
	return false;
}


//-------------------------------------------------
//  mute - mute sound output
//-------------------------------------------------

void sound_manager::mute(bool mute, u8 reason)
{
	if (mute)
		m_muted |= reason;
	else
		m_muted &= ~reason;
	set_attenuation(m_attenuation);
}


//-------------------------------------------------
//  reset - reset all sound chips
//-------------------------------------------------

void sound_manager::reset()
{
	// reset all the sound chips
	for (device_sound_interface &sound : sound_interface_iterator(machine().root_device()))
		sound.device().reset();
}


//-------------------------------------------------
//  pause - pause sound output
//-------------------------------------------------

void sound_manager::pause()
{
	mute(true, MUTE_REASON_PAUSE);
}


//-------------------------------------------------
//  resume - resume sound output
//-------------------------------------------------

void sound_manager::resume()
{
	mute(false, MUTE_REASON_PAUSE);
}


//-------------------------------------------------
//  config_load - read and apply data from the
//  configuration file
//-------------------------------------------------

void sound_manager::config_load(config_type cfg_type, util::xml::data_node const *parentnode)
{
	// we only care about game files
	if (cfg_type != config_type::GAME)
		return;

	// might not have any data
	if (parentnode == nullptr)
		return;

	// iterate over channel nodes
	for (util::xml::data_node const *channelnode = parentnode->get_child("channel"); channelnode != nullptr; channelnode = channelnode->get_next_sibling("channel"))
	{
		mixer_input info;
		if (indexed_mixer_input(channelnode->get_attribute_int("index", -1), info))
		{
			float defvol = channelnode->get_attribute_float("defvol", 1.0f);
			float newvol = channelnode->get_attribute_float("newvol", -1000.0f);
			if (newvol != -1000.0f)
				info.stream->set_user_gain(info.inputnum, newvol / defvol);
		}
	}
}


//-------------------------------------------------
//  config_save - save data to the configuration
//  file
//-------------------------------------------------

void sound_manager::config_save(config_type cfg_type, util::xml::data_node *parentnode)
{
	// we only care about game files
	if (cfg_type != config_type::GAME)
		return;

	// iterate over mixer channels
	if (parentnode != nullptr)
		for (int mixernum = 0; ; mixernum++)
		{
			mixer_input info;
			if (!indexed_mixer_input(mixernum, info))
				break;
			float newvol = info.stream->user_gain(info.inputnum);

			if (newvol != 1.0f)
			{
				util::xml::data_node *const channelnode = parentnode->add_child("channel", nullptr);
				if (channelnode != nullptr)
				{
					channelnode->set_attribute_int("index", mixernum);
					channelnode->set_attribute_float("newvol", newvol);
				}
			}
		}
}


//-------------------------------------------------
//  update - mix everything down to its final form
//  and send it to the OSD layer
//-------------------------------------------------

void sound_manager::update(void *ptr, int param)
{
	VPRINTF(("sound_update\n"));

	g_profiler.start(PROFILER_SOUND);

	// force all the speaker streams to generate the proper number of samples
	int samples_this_update = 0;
	for (speaker_device &speaker : speaker_device_iterator(machine().root_device()))
		speaker.mix(&m_leftmix[0], &m_rightmix[0], samples_this_update, (m_muted & MUTE_REASON_SYSTEM));

	// now downmix the final result
	u32 finalmix_step = machine().video().speed_factor();
	u32 finalmix_offset = 0;
	s16 *finalmix = &m_finalmix[0];
	int sample;
	for (sample = m_finalmix_leftover; sample < samples_this_update * 1000; sample += finalmix_step)
	{
		int sampindex = sample / 1000;

		// clamp the left side
		s32 samp = m_leftmix[sampindex];
		if (samp < -32768)
			samp = -32768;
		else if (samp > 32767)
			samp = 32767;
		finalmix[finalmix_offset++] = samp;

		// clamp the right side
		samp = m_rightmix[sampindex];
		if (samp < -32768)
			samp = -32768;
		else if (samp > 32767)
			samp = 32767;
		finalmix[finalmix_offset++] = samp;
	}
	m_finalmix_leftover = sample - samples_this_update * 1000;

	// play the result
	if (finalmix_offset > 0)
	{
		if (!m_nosound_mode)
			machine().osd().update_audio_stream(finalmix, finalmix_offset / 2);
		machine().osd().add_audio_to_recording(finalmix, finalmix_offset / 2);
		machine().video().add_sound_to_recording(finalmix, finalmix_offset / 2);
		if (m_wavfile != nullptr)
			wav_add_data_16(m_wavfile, finalmix, finalmix_offset);
	}

	// see if we ticked over to the next second
	attotime curtime = machine().time();
	bool second_tick = false;
	if (curtime.seconds() != m_last_update.seconds())
	{
		assert(curtime.seconds() == m_last_update.seconds() + 1);
		second_tick = true;
	}

	// iterate over all the streams and update them
	for (auto &stream : m_stream_list)
		stream->update_with_accounting(second_tick);

	// remember the update time
	m_last_update = curtime;

	// update sample rates if they have changed
	for (auto &stream : m_stream_list)
		stream->apply_sample_rate_changes();

	g_profiler.stop();
}
