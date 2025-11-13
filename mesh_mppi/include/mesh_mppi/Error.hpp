#pragma once

namespace mesh_mppi
{

enum class Error
{
    LETHAL_COST,
    OUT_OF_MAP,
    TRAJECTORY_REFERENCE_LENGTH_MISSMATCH,
    NO_VALID_COMMAND,
    PREDICTION_ERROR,
    INVALID_INPUT
};

};
