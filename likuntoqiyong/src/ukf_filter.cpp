#include "common_include.h"
#include "ukf_filter.h"


/*
 * UKF karman_filter
 * */
Eigen::MatrixXd sigmas(Eigen::MatrixXd x, Eigen::MatrixXd P, double c);
Eigen::MatrixXd Cholesky(Eigen::MatrixXd A);

void UKF::ukf( Eigen::MatrixXd& x, const Eigen::MatrixXd z)
{
    const int L=2*n+1;
    const double alpha = 1e-3;                                 //default, tunable
    const double ki = 0.0;                                     //default, tunable
    const double beta = 2.0;                                   //default, tunable
    const double lambda = (alpha*alpha)*(n+ki)-n;              //scaling factor
    double c = n+lambda;                                 //scaling factor

    /* weight equations are found in the upper part of http://www.cslu.ogi.edu/nsel/ukf/node6.html */
    Eigen::MatrixXd Wm(1,L);	//weights for means
    Eigen::MatrixXd Wc = Wm;	//weights for covariance
    Wm(0,0) = lambda/c;
    Wc(0,0) = lambda/c+(1-(alpha*alpha)+beta);
    for (unsigned int k=1; k<L; k++)
    {
        Wm(0,k) = 0.5/c;
        Wc(0,k) = 0.5/c;
    }
    c = sqrt(c);
    Eigen::MatrixXd X = sigmas(x, P, c);	//sigma points around x

    /* unscented transformation (ut) of process */
    Eigen::MatrixXd x1(n,1);
    Eigen::MatrixXd X1(n,L);
    for(unsigned int k=0; k<L; k++)
    {
        Eigen::MatrixXd Xcol(n,1);
        Eigen::MatrixXd X1col(n,1);	/* temp vectors, not used in matlab */

        for (unsigned int i=0; i<n; i++)
        {
            Xcol(i,0) = X(i,k);	// take out a column so that state_function can take it
        }
        X1col = state_function(Xcol);
        for (unsigned int i=0; i<n; i++)
        {
            x1(i,0) +=  Wm(0,k) * X1col(i,0);
        }
        for (unsigned int i=0; i<n; i++)
        {
            X1(i,k) = X1col(i,0);	// put back the output column
        }
    }
    Eigen::MatrixXd X2(n,L);
    for (unsigned int k=0; k<L; k++)
        for (unsigned int i=0; i<n; i++)
        {
            X2(i,k) = X1(i,k) - x1(i,0);	//X2.Column(k) = X1.Column(k) - x1;
        }
    Eigen::MatrixXd diagWm(L,L);
    for(unsigned int k=0; k<L; k++)
        diagWm(k,k) = Wm(0,k);
    Eigen::MatrixXd P1 = X2 * diagWm * X2.transpose() + Q;	/* ~ means transpose */

    /* unscented transformation (ut) of measurements */
    Eigen::MatrixXd z1(m,1);
    Eigen::MatrixXd Z1(m,L);
    for(unsigned int k=0; k<L; k++)
    {
        Eigen::MatrixXd X1col(n,1);
        Eigen::MatrixXd Z1col(m,1);	/* temp vectors, not used in matlab */

        for (unsigned int i=0; i<n; i++)
        {
            X1col(i,0) = X1(i,k);	// take out a column so that measurement_function can take it
        }
        Z1col = measurement_function(X1col);
        for (unsigned int i=0; i<m; i++)
        {
            z1(i,0) += Wm(0,k) * Z1col(i,0);
        }
        for (unsigned int i=0; i<m; i++)
        {
            Z1(i,k) = Z1col(i,0);	// put back the output column
        }
    }
    Eigen::MatrixXd Z2(m,L);
    for (unsigned int k=0; k<L; k++)
        for (unsigned int i=0; i<m; i++)
        {
            Z2(i,k) = Z1(i,k) - z1(i,0);	//Z2.Column(k) = Z1.Column(k) - z1;
        }
    Eigen::MatrixXd diagWc(L,L);
    for(unsigned int k=0; k<L; k++)
        diagWc(k,k) = Wc(0,k);
    Eigen::MatrixXd P2 = Z2 * diagWc * Z2.transpose() + R;

    Eigen::MatrixXd P12 = X2 * diagWc * Z2.transpose();	//transformed cross-covariance
    Eigen::MatrixXd K = P12 * P2.inverse();
    x = x1+K*(z-z1);                              //state update
    //cout << x << endl << K << endl;
    P = P1-K * P12.transpose();                                //covariance update
    //cout << P << endl << endl;
}

Eigen::MatrixXd sigmas(Eigen::MatrixXd x, Eigen::MatrixXd P, double c)
{
    const int n = P.rows();
    const int L=2*n+1;
    Eigen::MatrixXd Chol = Cholesky(P);
    Eigen::MatrixXd A = c * Chol;	/* doesn't need transpose here like matlab b/c Chol gives a lower (not upper) triangle matrix */

    Eigen::MatrixXd X(n,L);
    unsigned int k=0;
    {
        for (unsigned int i=0; i<n; i++)
        {
            X(i,k) = x(i,0);
        }
    }
    for(k=1; k<n+1; k++)
    {
        for (unsigned int i=0; i<n; i++)
        {
            X(i,k) = x(i,0) + A(i,k-1);
        }
    }
    for(k=n+1; k<L; k++)
    {
        for (unsigned int i=0; i<n; i++)
        {
            X(i,k) = x(i,0) - A(i,k-1-n);
        }
    }

    return X;
}

// returns a lower triangle matrix
Eigen::MatrixXd Cholesky(Eigen::MatrixXd A)
{
    const int n = A.rows();
    Eigen::MatrixXd Chol(n,n);
    double* s = new double[n];
    double ss;
    unsigned int i,j,k;
    for (j=0; j<n; j++)
    {
        if (j==0)
        {
            for (i=j; i<n; i++)
            {
                s[i]=A(i,j);
            }
        }
        if (j!=0)
        {
            for (i=j; i<n; i++)
            {
                ss=0.;
                for (k=0; k<=j-1; k++)
                {
                    ss+=Chol(i,k)*Chol(j,k);
                }
                s[i]=A(i,j)-ss;
            }
        }
        if (fabs(s[j])<0.000001)
        {
            likuncout << "ERROR: ukf.cpp Cholesky - Matrix not positive definite\n";
            delete [] s;
            return A;	// ERROR
        }
        for (i=j; i<n; i++)
        {
            Chol(i,j)=s[i]/sqrt(s[j]);
        }
    }
    delete [] s;
    return Chol;
}

//Matrix UKF::state_function (Matrix x)	// override this function with inherited UKF class
//{
//	cout << "UKF: Please form your own state function!" << endl;
//	return x;
//}
//
//Matrix UKF::measurement_function (Matrix x)	// override this function with inherited UKF class
//{
//	cout << "UKF: Please form your own measurement function!" << endl;
//	return x;
//}
