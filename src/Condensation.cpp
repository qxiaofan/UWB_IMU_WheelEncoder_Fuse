
#include "Condensation.h"

ConDensation::ConDensation(int dp, int numSamples, float flocking)
: DP(dp)
, numSamples(numSamples)
, flocking(flocking)
, rng(DP)
{
}

void ConDensation::initSampleSet(const  cv::Mat &lower, const  cv::Mat & upper, const  cv::Mat & dyna)
{
	CV_Assert((lower.type() == CV_32FC1) && (upper.type() == CV_32FC1));
	CV_Assert((lower.cols == DP) && (upper.cols == DP));
	CV_Assert((lower.rows == 1) && (upper.rows == 1));
	cv::Mat_<float>   lowerBound(lower); //{0,0,0}
	cv::Mat_<float>   upperBound(upper); //{2048,2048,2048}
	range = upper; // cache for reuse in correct() //{2048,2048,2048}

	// dyna might be empty (then we'll use an identity matrix), or a DP x DP x float transformation mat
	CV_Assert(dyna.empty() || ((dyna.rows == DP) && (dyna.rows == DP) && (dyna.type() == CV_32FC1)));
	dynamMatr = dyna.empty() ? cv::Mat_<float>::eye(DP, DP) : dyna; //dynamMatr为单位对角阵

	cumulative = cv::Mat_<float>::zeros(numSamples, 1); //累积权值
	samples = cv::Mat_<float>::zeros(numSamples, DP);//numSamples*3
	newSamples = cv::Mat_<float>::zeros(numSamples, DP);//新的粒子numSamples * 3
	randomSample = cv::Mat_<float>::zeros(1, DP);//randomSample {0,0,0}
	state = cv::Mat_<float>::zeros(1, DP);//state {0,0,0}
	mean = cv::Mat_<float>::zeros(1, DP);//mean {0,0,0}
	confidence = cv::Mat_<float>(numSamples, 1, 1.f / numSamples);//confidence {samples,1, 1.f/samples}

	for (int d = 0; d < DP; d++)
	{
		rng[d].set(cv::getTickCount(), lowerBound(d), upperBound(d)); //在每个维度生成一个随机数
	}

	// Generating the samples
	// 在每个维度，生成numSamples个随机数
	for (int s = 0; s < numSamples; s++)
	{
		for (int d = 0; d < DP; d++)
		{
			samples(s, d) = rng[d].uni();
		}
	}
}


void ConDensation::updateByTime()
{
	// Calculating the Mean 
	mean.setTo(0);
	float sum = 0.0f;
	for (int s = 0; s < numSamples; s++)
	{
		state = samples.row(s) * confidence(s);
		mean += state;
		sum += confidence(s);
		cumulative(s) = sum;
	}

	// Taking the new state vector from transformation of mean by dynamics matrix 
	mean /= sum;
	state = mean * dynamMatr; //根据均值，获得预测值
	//sum  /= numSamples;
	// Initialize the random number generator.
    //又开始新一轮生成新的粒子
	cv::RNG rngUp(cv::getTickCount());

	// We want a record of the span of the particle distribution. 
	// The resampled distribution is dependent on this quantity.
	// 我们想要记录下粒子分布的跨度，重采样的分布取决于此。
	std::vector<float> sampleMax(DP, FLT_MIN), sampleMin(DP, FLT_MAX);
	// Updating the set of random samples 
	// The algorithm of the original code always picked the last
	// sample, so was not really a weighted random re-sample.  It
	// wasn't really random, either, due to careless seeding of the
	// random number generation.

	// This version resamples according to the weights calculated by
	// the calling program and tries to be more consistent about
	// seeding the random number generator more carefully.
	//此版本根据调用程序计算的权重进行重新采样，并尝试更加谨慎地播种随机数生成器。
	for (int s = 0; s < numSamples; s++)
	{
		// Choose a random number between 0 and the sum of the particles' weights.
        // 选择介于0和粒子权重之和之间的一个随机数
		float randNumber = rngUp.uniform(0.0f, sum);

		// Use that random number to choose one of the particles.
		int j = 0;
		while ((cumulative(j) <= randNumber) && (j < numSamples - 1))
			//while( (cumulative(j) <= (float) s * sum) && (j<numSamples-1))
		{
			j++;
		}

		// Keep track of the max and min of the sample particles.
		// We'll use that to calculate the size of the distribution.
		for (int d = 0; d < DP; d++)
		{
			newSamples(s, d) = samples(j, d);
			sampleMax[d] = cv::max(sampleMax[d], newSamples(s, d));
			sampleMin[d] = cv::min(sampleMin[d], newSamples(s, d));

		}
	}

	// Reinitializes the structures to update samples randomly 
	for (int d = 0; d < DP; d++)
	{
		float diff = flocking * (sampleMax[d] - sampleMin[d]);
		if (0)
		{
			// This line may not be strictly necessary, but it prevents
			// the particles from congealing into a single particle in the
			// event of a poor choice of fitness (weighting) function.
			diff = cv::max(diff, 0.02f * newSamples(0, d));
		}
		else {
			// Rule 1 : reaching the target is the goal here, right ? 
			// * if we lost it         : swarm out  
			// * if target was reached : hog it .
			//判断是否到达目标这里？
			//如果丢失，则蜂拥而至；
			//如果已达到目标，则抱住。
			diff = cv::min(diff, flocking * (measure(d) - newSamples(0, d)));

		}

		// Re-seed and set the limits to the geometric extent of the distribution.
		// 重新设定种子，并为分布的几何范围设置极限
		rng[d].set(cv::getTickCount() + d, -diff, diff);

		// extra spin on the electronic roulette.(sic)
		rng[d].uni();
	}
	// Adding the random-generated vector to every projected vector in sample set

	//将随机生成的向量添加到样本集中的每个投影向量

	for (int s = 0; s < numSamples; s++)
	{
		cv::Mat_<float> r = newSamples.row(s) * dynamMatr;

		for (int d = 0; d < DP; d++)
		{
			samples(s, d) = r(d) + rng[d].uni();
		}
	}
}

//
//! adjust confidence based on euclidean distance and return predicted state
// 根据欧式距离调整置信度，并返回预测值
const  cv::Mat & ConDensation::correct(const  cv::Mat & measurement)
{
	measure = measurement; //测量值
	for (int s = 0; s < numSamples; s++)
	{
		double dist = 0;
		for (int d = 0; d < DP; d++)
		{
			float diff = (measure(d) - samples(s, d)) / range(d);

			dist += diff*diff;
		}
		confidence(s) = float(exp(-100.0f * sqrt(dist / (DP*DP)))); //每个粒子的置信度
	}
	updateByTime();
	return state;
}

