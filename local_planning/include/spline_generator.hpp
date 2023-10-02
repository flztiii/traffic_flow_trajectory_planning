#ifndef SPLINE_GENERATOR_HPP_
#define SPLINE_GENERATOR_HPP_

#include "spline.hpp"
#include "tk_spline/spline.h"

template <int N_DEG, int N_DIM>
class SplineGenerator {
 public:
    typedef Spline<N_DEG, N_DIM> SplineType;

    ErrorType GetCubicSplineBySampleInterpolation(const vec_Vecf<N_DIM>& samples, const std::vector<double>& para, SplineType* spline) {
        if (samples.size() < 3) {
            // printf("[SampleInterpolation]input sample size: %d.\n",
            //        static_cast<int>(samples.size()));
            return kWrongStatus;
        }
        if (samples.size() != para.size()) return kIllegalInput;
        if (N_DEG < 3) {
            printf("[CubicSplineBySampleInterpolation]Currently we just support interpolation >= cubic(3).\n");
            return kWrongStatus;
        }

        SplineType cubic_spline;
        // interpolate parameterization if too sparse
        // NOTE: the issue is caused by GetArcLengthByVecPosition
        // the current implementation is a fast version and reduces
        // the evaluation of spline as much as possible, but suffers
        // from the local minimum issue. To avoid the local minimum in practice
        // it is recommended to have a relatively dense parameterization
        // e.g, delta para < 3.5 * 2 = 7.0 m
        const double para_delta_threshold = 7.0;
        int num_samples = static_cast<int>(samples.size());
        vec_Vecf<N_DIM> interpolated_samples{samples[0]};
        std::vector<double> interpolated_para{para[0]};

        for (int i = 1; i < num_samples; i++) {
            double dis = para[i] - para[i - 1];
            if (dis > para_delta_threshold) {
                int n_inserted = static_cast<int>(dis / para_delta_threshold) + 1;
                double delta_para = dis / n_inserted;
                for (int n = 0; n < n_inserted; n++) {
                    double para_insert = para[i - 1] + (n + 1) * delta_para;
                    double partion = (n + 1) * delta_para / dis;
                    if (para_insert < para[i]) {
                        interpolated_para.push_back(para_insert);
                        Vecf<N_DIM> sample_insert;
                        for (int d = 0; d < N_DIM; d++) {
                            sample_insert[d] = (samples[i][d] - samples[i - 1][d]) * partion + samples[i - 1][d];
                        }
                        interpolated_samples.push_back(sample_insert);
                    }
                }
            }
            interpolated_para.push_back(para[i]);
            interpolated_samples.push_back(samples[i]);
        }

        cubic_spline.set_vec_domain(interpolated_para);
        for (int i = 0; i < N_DIM; i++) {
            std::vector<double> X, Y;
            for (int s = 0; s < static_cast<int>(interpolated_samples.size()); s++) {
                X.push_back(interpolated_para[s]);
                Y.push_back(interpolated_samples[s][i]);
            }

            tk::spline cubic_fitting;
            cubic_fitting.set_points(X, Y);

            for (int n = 0; n < cubic_fitting.num_pts() - 1; n++) {
                Vecf<N_DEG + 1> coeff = Vecf<N_DEG + 1>::Zero();
                for (int d = 0; d <= N_DEG; d++) {
                    if (d <= 3) {
                        coeff[N_DEG - d] = cubic_fitting.get_coeff(n, d) * fac(d);
                    } else {
                        coeff[N_DEG - d] = 0.0;
                    }
                }
                cubic_spline(n, i).set_coeff(coeff);
            }
        }
        (*spline) = std::move(cubic_spline);
        return kSuccess;
    }
};

#endif  // SPLINE_GENERATOR_HPP_