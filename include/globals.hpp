#pragma once

// Global variables that should be useful throughout the whole program

#define POLL_RATE 52
#define BATCH_SIZE_FILLED (3 * POLL_RATE) // 156
#define BATCH_SIZE 256 // Next highest power of 2
#define FREQUENCY_BIN_SIZE ((float)POLL_RATE / BATCH_SIZE)

#define DEBUG // Enables sanity checks and extra print statements

#define TELEPLOT // Enable print statements for Teleplot