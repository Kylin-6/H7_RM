#include "alg_rls.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int checks = 0;

static void Check(bool condition, const char *message)
{
    checks++;
    if (!condition)
    {
        fprintf(stderr, "FAIL: %s\n", message);
        exit(1);
    }
}

class Test_RLS : public Class_RLS
{
public:
    double Covariance_At(uint8_t row, uint8_t column) const
    {
        return Covariance[row][column];
    }
};

static void Contract()
{
    Test_RLS rls;
    float out[6] = {};
    float feature[2] = {1.0f, 2.0f};
    Check(!rls.Get_Initialized_Flag() && rls.Get_Dimension() == 0, "initial state");
    Check(!rls.Update(feature, 1.0f) && !rls.Get_Parameters(out, 6), "uninitialized operations");
    rls.Reset();

    float initial[2] = {2.0f, -3.0f};
    Check(rls.Init(2, 0.98f, 100.0f, initial), "valid initialization");
    Check(rls.Get_Parameters(out, 2) && out[0] == 2.0f && out[1] == -3.0f,
          "initial parameter copy");
    Check(!rls.Get_Parameters(out, 1) && !rls.Get_Parameters(NULL, 2), "output buffer bounds");
    Check(rls.Get_Parameter(2) == 0.0f, "invalid index");

    double covariance = rls.Covariance_At(0, 0);
    float bad_initial[2] = {1.0f, NAN};
    Check(!rls.Init(0, 1.0f, 1.0f), "zero dimension");
    Check(!rls.Init(7, 1.0f, 1.0f), "excess dimension");
    Check(!rls.Init(2, 0.0f, 1.0f) && !rls.Init(2, 1.01f, 1.0f), "forgetting range");
    Check(!rls.Init(2, 1.0f, 0.0f) && !rls.Init(2, 1.0f, INFINITY), "covariance range");
    Check(!rls.Init(2, 1.0f, 1.0f, bad_initial), "initial parameter finite");
    Check(rls.Get_Dimension() == 2 && rls.Get_Parameter(0) == 2.0f &&
          rls.Covariance_At(0, 0) == covariance, "failed Init keeps configuration and state");

    feature[1] = NAN;
    Check(!rls.Update(feature, 1.0f), "NaN feature");
    feature[1] = INFINITY;
    Check(!rls.Update(feature, 1.0f), "infinite feature");
    feature[1] = 2.0f;
    Check(!rls.Update(NULL, 1.0f) && !rls.Update(feature, NAN) &&
          !rls.Update(feature, INFINITY), "invalid measurement and pointer");
    Check(rls.Get_Parameter(0) == 2.0f && rls.Get_Parameter(1) == -3.0f &&
          rls.Covariance_At(0, 0) == covariance, "invalid Update keeps state");

    Check(rls.Update(feature, 4.0f), "valid update");
    Check(rls.Get_Parameter(0) != 2.0f, "estimate changed");
    rls.Reset();
    Check(rls.Get_Parameter(0) == 2.0f && rls.Get_Parameter(1) == -3.0f &&
          rls.Covariance_At(0, 0) == covariance &&
          rls.Covariance_At(0, 1) == 0.0, "Reset restores initial parameters and covariance");
}

static void Convergence()
{
    Test_RLS rls;
    const float truth[6] = {1.25f, -2.5f, 0.125f, 4.0f, -0.75f, 3.25f};
    Check(rls.Init(6, 1.0f, 1000.0f), "six-dimensional Init");
    for (int cycle = 0; cycle < 30; cycle++)
    {
        for (int index = 0; index < 6; index++)
        {
            float feature[6] = {};
            feature[index] = 1.0f;
            Check(rls.Update(feature, truth[index]), "basis observation");
        }
    }
    float estimate[6] = {};
    Check(rls.Get_Parameters(estimate, 6), "Get_Parameters");
    for (int i = 0; i < 6; i++)
    {
        Check(fabsf(estimate[i] - truth[i]) < 0.0002f, "known model convergence");
        for (int j = 0; j < 6; j++)
        {
            Check(rls.Covariance_At(i, j) == rls.Covariance_At(j, i), "covariance symmetry");
        }
    }
}

static void Analytic()
{
    Test_RLS rls;
    float feature[2] = {1.0f, 2.0f};
    Check(rls.Init(2, 1.0f, 10.0f), "analytic Init");
    Check(rls.Update(feature, 3.0f), "analytic update");
    Check(fabs(rls.Get_Parameter(0) - 30.0 / 51.0) < 1e-7, "analytic parameter 0");
    Check(fabs(rls.Get_Parameter(1) - 60.0 / 51.0) < 1e-7, "analytic parameter 1");
    Check(fabs(rls.Covariance_At(0, 0) - (10.0 - 100.0 / 51.0)) < 1e-12,
          "analytic covariance 00");
    Check(fabs(rls.Covariance_At(0, 1) + 200.0 / 51.0) < 1e-12 &&
          rls.Covariance_At(0, 1) == rls.Covariance_At(1, 0),
          "analytic covariance symmetry");
    Check(fabs(rls.Covariance_At(1, 1) - (10.0 - 400.0 / 51.0)) < 1e-12,
          "analytic covariance 11");
}

static void Forgetting()
{
    Class_RLS fixed;
    Class_RLS adaptive;
    float feature = 1.0f;
    Check(fixed.Init(1, 1.0f, 10.0f) && adaptive.Init(1, 0.95f, 10.0f),
          "comparison Init");
    for (int i = 0; i < 200; i++)
    {
        Check(fixed.Update(&feature, 2.0f) && adaptive.Update(&feature, 2.0f),
              "first regime observation");
    }
    Check(fabsf(fixed.Get_Parameter(0) - 2.0f) < 0.01f &&
          fabsf(adaptive.Get_Parameter(0) - 2.0f) < 0.01f, "first regime learned");
    for (int i = 0; i < 25; i++)
    {
        Check(fixed.Update(&feature, 5.0f) && adaptive.Update(&feature, 5.0f),
              "second regime observation");
    }
    Check(fabsf(adaptive.Get_Parameter(0) - 5.0f) <
          fabsf(fixed.Get_Parameter(0) - 5.0f) * 0.6f,
          "forgetting follows changed model faster");
}

static void Numerical_Failure()
{
    Test_RLS rls;
    float feature = 0.0f;
    Check(rls.Init(1, 1.0e-30f, 1.0e30f), "extreme finite Init");
    bool failed = false;
    for (int i = 0; i < 20; i++)
    {
        double before_parameter = rls.Get_Parameter(0);
        double before_covariance = rls.Covariance_At(0, 0);
        if (!rls.Update(&feature, 0.0f))
        {
            Check(rls.Get_Parameter(0) == before_parameter &&
                  rls.Covariance_At(0, 0) == before_covariance,
                  "numerical failure keeps state");
            failed = true;
            break;
        }
    }
    Check(failed, "non-finite covariance detected");
    rls.Reset();
    Check(rls.Covariance_At(0, 0) == (double)1.0e30f, "Reset after numerical failure");
}

static void Positive_Definite()
{
    Test_RLS rls;
    const float feature[2] = {1.0e20f, 1.0e20f};
    Check(rls.Init(2, 1.0f, 1.0e20f), "ill-conditioned but legal Init");
    double previous_covariance = rls.Covariance_At(0, 0);
    Check(!rls.Update(feature, 1.0f), "singular rounded candidate rejected");
    Check(rls.Get_Parameter(0) == 0.0f && rls.Get_Parameter(1) == 0.0f &&
          rls.Covariance_At(0, 0) == previous_covariance &&
          rls.Covariance_At(1, 1) == previous_covariance &&
          rls.Covariance_At(0, 1) == 0.0 &&
          rls.Covariance_At(1, 0) == 0.0,
          "non-positive-definite candidate keeps entire old state");

    const float ordinary_feature[2] = {1.0e-10f, 0.0f};
    Check(rls.Update(ordinary_feature, 2.0f), "valid update after rejected candidate");
    Check(rls.Covariance_At(0, 0) > 0.0 && rls.Covariance_At(1, 1) > 0.0,
          "positive covariance after recovery");
}

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        fprintf(stderr, "usage: rls_tests contract|analytic|convergence|forgetting|numerical_failure|positive_definite\n");
        return (1);
    }
    if (strcmp(argv[1], "contract") == 0) Contract();
    else if (strcmp(argv[1], "analytic") == 0) Analytic();
    else if (strcmp(argv[1], "convergence") == 0) Convergence();
    else if (strcmp(argv[1], "forgetting") == 0) Forgetting();
    else if (strcmp(argv[1], "numerical_failure") == 0) Numerical_Failure();
    else if (strcmp(argv[1], "positive_definite") == 0) Positive_Definite();
    else return (1);
    printf("PASS %s: %d checks\n", argv[1], checks);
    return (0);
}
