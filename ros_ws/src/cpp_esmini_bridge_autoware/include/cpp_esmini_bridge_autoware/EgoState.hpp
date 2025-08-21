#pragma once

typedef enum EgoState {
    INITIALIZING,
    PLANNING,
    WAITING_FOR_ENGAGE,
    DRIVING,
    FINALIZED,
    UNKNOWN,
} EgoState;
