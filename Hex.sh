#Physical properties
    #Lengths in meters
        export HEX_BASE_COXA_LENGTH=0.6
        export HEX_COXA_LENGTH=0.6
        export HEX_FEMUR_LENGTH=0.555
        export HEX_TIBIA_LENGTH=0.755

    # Masses in kg
        export HEX_COVER_TOP_MASS=0.1
        export HEX_BODY_TOP_MASS=0.7
        export HEX_BOARD_BRACKET_MASS=0.7
        export HEX_BODY_BOTTOM_MASS=0.6
        export HEX_COXA_MASS=0.6
        export HEX_FEMUR_MASS=0.6
        export HEX_TIBIA_MASS=0.6

    # Joints limits in radians
        export HEX_COXA_JOINT_LIMIT_LOWER=-0.707
        export HEX_COXA_JOINT_LIMIT_UPPER=0.707
        export HEX_FEMUR_JOINT_LIMIT_LOWER=-0.707
        export HEX_FEMUR_JOINT_LIMIT_UPPER=0.707
        export HEX_TIBIA_JOINT_LIMIT_LOWER=-2.3
        export HEX_TIBIA_JOINT_LIMIT_UPPER=0

        # export HEX_COXA_JOINT_LIMIT_LOWER=0
        # export HEX_COXA_JOINT_LIMIT_UPPER=6.28
        # export HEX_FEMUR_JOINT_LIMIT_LOWER=0
        # export HEX_FEMUR_JOINT_LIMIT_UPPER=6.28
        # export HEX_TIBIA_JOINT_LIMIT_LOWER=0
        # export HEX_TIBIA_JOINT_LIMIT_UPPER=6.28

    # Joints stiffness in Nm/rad

        export HEX_COXA_JOINT_DAMPING=60.0
        export HEX_FEMUR_JOINT_DAMPING=60.0
        export HEX_TIBIA_JOINT_DAMPING=0.7
        export HEX_COXA_JOINT_FRICTION=4.0
        export HEX_FEMUR_JOINT_FRICTION=4.0
        export HEX_TIBIA_JOINT_FRICTION=0.1

    # Joints effor and velocity
        export HEX_BASE_COXA_EFFORT=50.0
        export HEX_COXA_FEMUR_EFFORT=50.0
        export HEX_FEMUR_TIBIA_EFFORT=50.0
        export HEX_BASE_COXA_VELOCITY=1.0
        export HEX_COXA_FEMUR_VELOCITY=1.0
        export HEX_FEMUR_TIBIA_VELOCITY=1.0

# TODO set the dynamic path
# Gazebo Model Path
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/hex/Hexapod/src/gazebo_models_worlds_collection/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/home/hex/Hexapod/src/gazebo_models_worlds_collection/worlds
