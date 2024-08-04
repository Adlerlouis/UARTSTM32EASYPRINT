
#define SAMPLEFILTER_TAP_NUM 171
typedef struct {
  int32_t history[SAMPLEFILTER_TAP_NUM];
  unsigned int last_index;
} SampleFilter;

static double filter_taps[SAMPLEFILTER_TAP_NUM] = {
		//valores de tu filtro

		0.025608316,
		0.074969429,
		0.085706851,
		0.011029593,
	   -0.158623749,
	   -0.242172976,
	   -0.027460683,
	    0.470665678,
	    0.768846299,
	    0.293917554,
		-0.955258975,
		-1.911379312,
		-0.957033132,
		2.800562803,
		8.595303988,
		14.18913147,
		17.40944544,
		17.58566482,
		15.84955967,
		13.97177021,
		13.06795441,
		13.11278042,
		13.60304449,
		14.2605039,
};
void SampleFilter_init(SampleFilter*f);
void SampleFilter_put(SampleFilter* f, int32_t input);
double SampleFilter_get(SampleFilter*f);


