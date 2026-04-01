#ifndef CODEFREE_COMM_CONFIG_H
#define CODEFREE_COMM_CONFIG_H

/** Common configuration */
#define CODEFREE_COMM_RECEIVE_TIMEOUT_MS 1
#define USE_CODEFREE_COMM_I2C

/** UART configuration */
#ifdef USE_CODEFREE_COMM_UART
#else
/** I2C configuration */
#endif

#endif /* CODEFREE_COMM_CONFIG_H */
/* EOF */
