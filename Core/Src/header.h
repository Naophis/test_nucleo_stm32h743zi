#ifndef HEADER_H_
#define HEADER_H_

typedef struct
{
    int right;
    int left;
} t_Encorder;

typedef struct
{
    int right45;
    int right90;
    int left45;
    int left90;
    int front;
} t_LightSensor;

#define GYRO_BUF 5
typedef struct
{
    int gyro_raw_data[GYRO_BUF];
    int gyro_data[GYRO_BUF];
} t_Gyro;

typedef struct
{
    t_LightSensor light_sensor;
    t_Gyro gryo;
    t_Encorder encorder;
    double ego_angle;
    double ego_velocity;
} t_SensorData;

typedef struct
{
    t_Gyro gryo;
} t_Target;

#endif