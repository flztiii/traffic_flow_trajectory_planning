#ifndef POLYNOMIAL_HPP_
#define POLYNOMIAL_HPP_

#include "calculation.hpp"

template <int N_DEG>
class Polynomial {
 public:
    typedef Vecf<N_DEG + 1> VecNf;
    enum { NeedsToAlign = (sizeof(VecNf) % 16) == 0 };

    Polynomial() { 
        set_zero(); 
    }
    
    Polynomial(const VecNf& coeff) {
        this->coeff_ = coeff;
        update();
    }

    /**
     * @brief Return coefficients of the polynomial
     */
    VecNf coeff() const { 
        return this->coeff_; 
    }

    /**
     * @brief Set coefficients of the polynomial
     */
    void set_coeff(const VecNf& coeff) {
        this->coeff_ = coeff;
        update();
    }

    void update() {
        for (int i = 0; i < N_DEG + 1; i++) {
            this->coeff_normal_order_[i] = this->coeff_[N_DEG - i] / fac(i);
        }
    }

    /**
     * @brief Set coefficients of the polynomial to zero
     */
    void set_zero() {
        this->coeff_.setZero();
        this->coeff_normal_order_.setZero();
    }

    /**
     * @brief Return position of polynomial with respect to parameterization
     * @param s value of polynomial parameterization
     */
    inline double evaluate(const double& s, const int& d) const {
        // Use horner's rule for quick evaluation
        double p = this->coeff_(0) / fac(N_DEG - d);
        for (int i = 1; i <= N_DEG - d; i++) {
            p = (p * s + this->coeff_(i) / fac(N_DEG - i - d));
        }
        return p;
    }

    inline double evaluate(const double& s) const {
        // Use horner's rule for quick evaluation
        // note that this function is much faster than evaluate(s, 0);
        double p = this->coeff_normal_order_[N_DEG];
        for (int i = 1; i <= N_DEG; i++) {
            p = (p * s + this->coeff_normal_order_[N_DEG - i]);
        }
        return p;
    }

    inline double J(double s, int d) const {
        if (d == 3) {
            // integration of squared jerk
            return this->coeff_(0) * this->coeff_(0) / 20.0 * pow(s, 5) + this->coeff_(0) * this->coeff_(1) / 4 * pow(s, 4) + (this->coeff_(1) * this->coeff_(1) + this->coeff_(0) * this->coeff_(2)) / 3 * pow(s, 3) + this->coeff_(1) * this->coeff_(2) * s * s + this->coeff_(2) * this->coeff_(2) * s;
        } else if (d == 2) {
            // integration of squared accleration
            return this->coeff_(0) * this->coeff_(0) / 252 * pow(s, 7) + this->coeff_(0) * this->coeff_(1) / 36 * pow(s, 6) + (this->coeff_(1) * this->coeff_(1) / 20 + this->coeff_(0) * this->coeff_(2) / 15) * pow(s, 5) + (this->coeff_(0) * this->coeff_(3) / 12 + this->coeff_(1) * this->coeff_(2) / 4) * pow(s, 4) + (this->coeff_(2) * this->coeff_(2) / 3 + this->coeff_(1) * this->coeff_(3) / 3) * pow(s, 3) + this->coeff_(2) * this->coeff_(3) * s * s + this->coeff_(3) * this->coeff_(3) * s;
        } else {
            throw;
        }
        return 0.0;
    }

    MatNf<6> GetAInverse(double t) {
        MatNf<6> A;
        A << 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
            0.0, 2.0, 0.0, 0.0, pow(t, 5), pow(t, 4), pow(t, 3), t * t, t, 1,
            5.0 * pow(t, 4), 4.0 * pow(t, 3), 3.0 * t * t, 2.0 * t, 1.0, 0.0,
            20.0 * pow(t, 3), 12.0 * t * t, 6.0 * t, 2.0, 0.0, 0.0;
        return A.inverse();
    }

    /**
     * @brief Generate jerk-optimal primitive with boundary condition
     * @note The polynomial should have degree >= quintic (5)
     * @param p1, x0 position
     * @param dp1, x0 velocity
     * @param ddp1, x0 acceleration
     * @param p2, x1 position
     * @param dp2, x1 velocity
     * @param ddp2, x1 acceleration
     * @param S, duration
     */
    void GetJerkOptimalConnection(const double p1, const double dp1, const double ddp1, const double p2, const double dp2, const double ddp2, const double S) {
        assert(N_DEG >= 5);
        Vecf<6> b;
        b << p1, dp1, ddp1, p2, dp2, ddp2;

        this->coeff_.setZero();

        // NOTE: fix the singularity of S=0 caused
        // by stopping traj
        if (S < kEPS) {
            Vecf<6> c;
            c << 0.0, 0.0, 0.0, ddp1, dp1, p1;
            this->coeff_.template segment<6>(N_DEG - 5) = c;
            return;
        }

        MatNf<6> A_inverse;
        if (!LookUpCache(S, &A_inverse)) {
            A_inverse = GetAInverse(S);
        }

        auto coeff = A_inverse * b;
        this->coeff_[N_DEG - 5] = coeff(0) * fac(5);
        this->coeff_[N_DEG - 4] = coeff(1) * fac(4);
        this->coeff_[N_DEG - 3] = coeff(2) * fac(3);
        this->coeff_[N_DEG - 2] = coeff(3) * fac(2);
        this->coeff_[N_DEG - 1] = coeff(4) * fac(1);
        this->coeff_[N_DEG - 0] = coeff(5);
        update();
        // coeff_.template segment<6>(N_DEG - 5) = A_inverse * b;;
    }

    /**
     * @Get look up cached A inverse
     */
    bool LookUpCache(const double S, MatNf<6>* A_inverse) {
        auto it = kTableAInverse.find(S);
        if (it != kTableAInverse.end()) {
            *A_inverse = it->second;
            return true;
        } else {
            return false;
        }
    }

    /**
     * @brief Debug function
     */
    void print() const {
        std::cout << std::fixed << std::setprecision(7) << this->coeff_.transpose() << std::endl;
    }

 private:
    VecNf coeff_;
    VecNf coeff_normal_order_;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(NeedsToAlign)
    std::map<double, MatNf<6>, std::less<double>, Eigen::aligned_allocator<std::pair<const double, MatNf<6>>>> kTableAInverse{
        {3.0, GetAInverse(3.0)}, {3.5, GetAInverse(3.5)},
        {4.0, GetAInverse(4.0)}, {4.5, GetAInverse(4.5)},
        {5.0, GetAInverse(5.0)},
    };
};

template <int N_DEG, int N_DIM>
class PolynomialND {
 public:
    PolynomialND() {}

    PolynomialND(const std::array<Polynomial<N_DEG>, N_DIM>& polys) {
        this->polys_ = polys;
    }

    /**
     * @brief Return a reference to a certain 1D polynomial
     * @param j index of the dimension
     */
    Polynomial<N_DEG>& operator[](int j) {
        assert(j < N_DIM);
        return this->polys_[j];
    }

    /**
     * @brief Return evaluated vector
     * @param s evaluation point
     * @param d derivative to take
     */
    inline void evaluate(const double s, int d, Vecf<N_DIM>* vec) const {
        for (int i = 0; i < N_DIM; i++) {
            (*vec)[i] = this->polys_[i].evaluate(s, d);
        }
    }

    inline void evaluate(const double s, Vecf<N_DIM>* vec) const {
        for (int i = 0; i < N_DIM; i++) {
            (*vec)[i] = this->polys_[i].evaluate(s);
        }
    }

    void print() const {
        for (int i = 0; i < N_DIM; i++) {
            this->polys_[i].print();
        }
    }

 private:
    std::array<Polynomial<N_DEG>, N_DIM> polys_;
};

#endif  // POLYNOMIAL_HPP_