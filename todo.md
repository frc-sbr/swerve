OVERARCHING GOAL = robot

- robot
    - swerve drivetrain
        - swerve modules
            - motor controllers (spark maxes)
                - should be handled by CANSparkMax
                - I need to figure out how tf to get ^^
            - retrieving encoder data
                - neo-internal **should** be as easy as connecting encoderout-->sparkmax-->__using some helper function in the spark maxes???__
                - thriftybot **should** be as easy as using the AnalogEncoder helper library
                    - see [this](https://drive.google.com/file/d/13NQDb2Zo_SesC1MDLzDeBgxTNhccqtUq/view) for more information
            - gyro data
                - research mx2 library
    - general
        - figure out wiring/ports

- i am copying (link)[https://www.youtube.com/watch?v=0Xi9yb1IMyA&t=381s] and am at 8:25