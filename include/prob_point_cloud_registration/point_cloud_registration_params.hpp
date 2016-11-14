#ifndef PROB_POINT_CLOUD_REGISTRATION_POINT_CLOUD_REGISTRATION_PARAMS_H
#define PROB_POINT_CLOUD_REGISTRATION_POINT_CLOUD_REGISTRATION_PARAMS_H

namespace prob_point_cloud_registration
{
struct PointCloudRegistrationParams
{
    int max_neighbours;
    double dof;
    int dimension = 3;
    double initial_rotation[4] = {1, 0, 0, 0};
    double initial_translation[3] = {0, 0, 0};
};
}

#endif
