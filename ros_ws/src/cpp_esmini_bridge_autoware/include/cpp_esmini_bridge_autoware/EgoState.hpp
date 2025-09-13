#pragma once

typedef enum class EgoState {
    INITIALIZING,
    PLANNING,
    WAITING_FOR_ENGAGE,
    DRIVING,
    FINALIZED,
    UNKNOWN,
} EgoState;
