#pragma once

/**
 * Rotormast V22 Osprey mixer.
 *
 * Collects four inputs (roll, pitch, yaw, thrust) and mixes them to servo commands
 * for swash plate tilting and throttle- and pitch curves.
 */
#include <mixer/MixerBase/Mixer.hpp>

class V22OspreyServo
{
public:
	V22OspreyServo() :
		min(-1.0f),
		max(1.0f),
		center(0.0f)
	{}

	//  FUNCTION TO CHANGE -1 --> 1 RAW VALUE TO RESTRICTED AND OFFSET VALUE FOR OUTPUT
	float scaleToLimits(float valueA)
	{
		if(valueA < 0.0f)
			return (center - min) * valueA + center;
		else
			return (max - center) * valueA + center;
	}

public:
	float min;
	float max;
	float center;
};


class V22OspreyMixer : public Mixer
{
public:
    enum {	NUM_VOL_POINTS=3,
			NUM_COL_POINTS=5 };

    enum {  AIL,
            ELEV,
            RUD,
            THR,
            COL,
            NACELLE,
            NUM_CONTROLS
         };

    enum {  LEFT_S0,
            LEFT_S1,
            LEFT_ST,
            LEFT_SN,
            RIGHT_S0,
            RIGHT_S1,
            RIGHT_ST,
            RIGHT_SN,
            NUM_MAIN_SERVOS
         };

    /**
     * Constructor.
     *
     * @param control_cb		Callback invoked to read inputs.
     * @param cb_handle		Passed to control_cb.
     */
    V22OspreyMixer(ControlCallback control_cb,
					uintptr_t cb_handle,
					uint8_t setupFlagA,
					float *rollVolCurveA,
					float *foreAftVolCurveA,
					float *yawVolCurveA,
					float colVolumeA,
					float *leftColCurveA,
					float *rightColCurveA,
					V22OspreyServo servo_dataA[NUM_MAIN_SERVOS]);
    virtual ~V22OspreyMixer();

	V22OspreyMixer(const V22OspreyMixer &) = delete;
	V22OspreyMixer &operator=(const V22OspreyMixer &) = delete;
	V22OspreyMixer(V22OspreyMixer &&) = delete;
	V22OspreyMixer &operator=(V22OspreyMixer &&) = delete;
    /**
     * Factory method.
     *
     * Given a pointer to a buffer containing a text description of the mixer,
     * returns a pointer to a new instance of the mixer.
     *
     * @param control_cb		The callback to invoke when fetching a
     *				control value.
     * @param cb_handle		Handle passed to the control callback.
     * @param buf			Buffer containing a text description of
     *				the mixer.
     * @param buflen		Length of the buffer in bytes, adjusted
     *				to reflect the bytes consumed.
     * @return			A new V22OspreyMixer instance, or nullptr
     *				if the text format is bad.
     */
    static V22OspreyMixer *from_text(Mixer::ControlCallback control_cb,
                                     uintptr_t cb_handle,
                                     const char *buf,
                                     unsigned &buflen);

    unsigned mix(float *outputs, unsigned space) override;
    void groups_required(uint32_t &groups) override;

    unsigned set_trim(float trim) override { return NUM_MAIN_SERVOS; }
	unsigned get_trim(float *trim) override { return NUM_MAIN_SERVOS; }

    float computeValueFromCurve(float pointA, float minRangeA, float maxRangeA, float *curveA, uint32_t numPointsA);

private:
	uint8_t setup_flag;
    float rollVolCurve[NUM_VOL_POINTS];
	float foreAftVolCurve[NUM_VOL_POINTS];
	float yawVolCurve[NUM_VOL_POINTS];
	float colVolume;
	float leftColCurve[NUM_COL_POINTS];
	float rightColCurve[NUM_COL_POINTS];
	V22OspreyServo servos[NUM_MAIN_SERVOS];
};
