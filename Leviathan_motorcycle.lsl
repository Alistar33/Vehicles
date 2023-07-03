// Leviathan Bear Motorcycle example script
//
// Originally written by Cory Linden. 
// Then modified and tweaked by Andrew Linden for the script_library.
// Finally tweaked and updated by Leviathan Linden.
//
// Root prim should be oriented such that its local X-, Y- and Z-axes are
// parallel to forward, left, and up respectively.
//
// Sound triggers are commented out but not removed, so if you
// want sounds, just add the sounds to the cycle's contents and uncomment
// the triggers.
//
// Be careful when changing parameters.  Some of them can be very 
// sensitive to change.  I recommend: change only one at a time, and test
// after each modification.
//
// The geometry of the motorcycle itself can have significant impact on 
// whether it in a straight line when not trying to turn.  For best results 
// use a symmetric design with as wide of a base as you can tolerate.
// this will tend to keep the motorcycle upright will avoid spurious banking
// effects when it tilts from collisions.

// These are globals only for convenience (so when you need to modify
// them you need only make a single change).  There are other magic numbers 
// below that have not yet been pulled into globals.  Some of these numbers
// are very sensitive: be careful when modifying them.
float gMaxTurnSpeed = 12;
float gMaxWheelieSpeed = 5;
float gMaxFwdSpeed = 30;
float gMaxBackSpeed = -10;
float gAngularRamp = 0.17;
float gLinearRamp = 0.2;

// These are true globals whose values are "accumulated" over  multiple control()
// callbacks.  The accumulation behavior interacts with the motor timescale settings
// to produce the ramp-up of vehicle speed and also braking.
float gBank = 0.0;
vector gLinearMotor = <0, 0, 0>;
vector gAngularMotor = <0, 0, 0>;

default
{
    state_entry()
    {
        // init stuff that never changes
        llSetSitText("Ride");
        llCollisionSound("", 0.0);
        llSitTarget(<0.6, 0.05, 0.20>, ZERO_ROTATION);
        llSetCameraEyeOffset(<-6.0, 0.0, 1.0>);
        llSetCameraAtOffset(<3.0, 0.0, 1.0>);
    
        // create the vehicle
        // Using VEHICLE_TYPE_CAR sets some default flags and parameters,
        // but we customize them below.
        llSetVehicleType(VEHICLE_TYPE_CAR);
        
        // VEHICLE_FLAG_LIMIT_MOTOR_UP = linear motor cannot have non-zero world-frame UP component
        //    (e.g. it is a "ground vehicle" that should not "fly into the sky").
        // VEHICLE_FLAG_LIMIT_ROLL_ONLY = modifies the behavior of the vertidal attractor
        //    (see VEHICLE_VERTICAL_ATTRACTION fields below) to only limit the vehicle's "roll"
        //    (e.g. rotation about the forward axis) and not pitch.
        llSetVehicleFlags(VEHICLE_FLAG_LIMIT_MOTOR_UP | VEHICLE_FLAG_LIMIT_ROLL_ONLY);
        
        // LINEAR_DEFLECTION coefficients determine the ability of the vehicle to divert
        //    sideways velocity into forward velocity.  For simplest tuning: always set the
        // DEFLECTION_EFFICIENCY to 1.0 and only adjust the timescale. Short timescales
        //     make the deflection strong and long timescales make it weak.
        llSetVehicleFloatParam(VEHICLE_LINEAR_DEFLECTION_EFFICIENCY, 1.0);
        llSetVehicleFloatParam(VEHICLE_LINEAR_DEFLECTION_TIMESCALE, 0.5);
        
        // Similarly, the ANGULAR_DEFLECTION coefficients determine the ability of the
        // vehicle to reorient itself to point forward in whatever direction it is moving.
        // In other words: is the vehicle a "dart" with fins that help it point forward
        // or is it a round ball with no orientation preference to its world-frame linear
        // velocity?
        // For simplest tuning: always set the DEFLECTION_EFFICIENCY to 1.0
        // and only adjust the timescale.  Short timescales make the deflection
        // strong and long timescales make it weak.
        llSetVehicleFloatParam(VEHICLE_ANGULAR_DEFLECTION_EFFICIENCY, 1.0);  
        llSetVehicleFloatParam(VEHICLE_ANGULAR_DEFLECTION_TIMESCALE, 1.4);
        
        // Motor timescales determine how quickly the motor achieves the desired
        // velocity.  Short timescales make it ramp up very quickly, long timescales
        // make it slow.  As a rule of thumb: estimate the time you'd like the motor
        // to achieve its full speed then set the timescale to 1/3 of that.
        llSetVehicleFloatParam(VEHICLE_LINEAR_MOTOR_TIMESCALE, 0.8);
        llSetVehicleFloatParam(VEHICLE_ANGULAR_MOTOR_TIMESCALE, 0.01);        
        
        // Motor decay timescales determine how quickly the motor's desired speed
        // decays toward zero.  This timescale cannot be set longer than 120 seconds,
        // the idea being: you can't just set a vehicle's velocity and expect it to
        // move forward forever; you must continually poke it in order to keep it going.
        // This to prevent a trivial runaway vehicle.  Here we use relatively short
        // timescales to help the vehicle brake when controls stop.
        llSetVehicleFloatParam(VEHICLE_ANGULAR_MOTOR_DECAY_TIMESCALE, 0.35);
        llSetVehicleFloatParam(VEHICLE_LINEAR_MOTOR_DECAY_TIMESCALE, 0.4);

        // Long timescales make friction weak; short timescales make it strong.
        // The friction can be asymmetric: different values for local X, Y, and Z axes
        // (e.g. forward, left, and up).
        llSetVehicleVectorParam(VEHICLE_LINEAR_FRICTION_TIMESCALE, <1000, 100, 1000>);
        llSetVehicleVectorParam(VEHICLE_ANGULAR_FRICTION_TIMESCALE, <100, 20, 100>);

        // The "vertial attractor" is a spring-like behavior that keeps the vehicle upright.
        llSetVehicleFloatParam(VEHICLE_VERTICAL_ATTRACTION_EFFICIENCY, 1.0);
        llSetVehicleFloatParam(VEHICLE_VERTICAL_ATTRACTION_TIMESCALE, 0.44);

        // The banking behavior introduces a torque about the world-frame UP axis when the
        // vehicle "rolls" about its forward axis..
        //
        // The VEHICLE_BANKING_EFFICIENCY is a multiplier on this effect, and can be > 1.0.
        //    VEHICLE_BANKING_EFFICIENCY = 0.0 --> no banking
        //    VEHICLE_BANKING_EFFICIENCY = 1.0 --> yes banking        
        llSetVehicleFloatParam(VEHICLE_BANKING_EFFICIENCY, 1.0);
        
        // Short VEHICLE_BANKING_TIMESCALE makes banking very effective,
        // long values makes it weak.
        llSetVehicleFloatParam(VEHICLE_BANKING_TIMESCALE, 0.01);      
        
        // VEHICLE_BANKING_MIX is a sliding weight on the banking effect which was
        // supposed to work as follows:
        //    mix=0.0 --> always on (just tilt the vehicle and it will magically turn)
        //    mix=1.0 --> only effective when there is a non-zero forward velocity 
        //    (e.g. unable to turn by banking when vehicle is at rest).    
        llSetVehicleFloatParam(VEHICLE_BANKING_MIX, 0.9);
        
        // This motorcycle doesn't use the "hover" feature, but here is some code
        // for experimentation.
        //llSetVehicleFloatParam(VEHICLE_HOVER_HEIGHT, 2.0);
        //llSetVehicleFloatParam(VEHICLE_HOVER_TIMESCALE, 1.0);
        //llSetVehicleFloatParam(VEHICLE_HOVER_EFFICIENCY, 0.5);
    }

    changed(integer change)
    {
        if (change & CHANGED_LINK)
        {
            key agent = llAvatarOnSitTarget();
            if (agent)
            {
                if (agent != llGetOwner())
                {
                    // not the owner ==> boot off
                    llSay(0, "You aren't the owner");
                    llUnSit(agent);
                    llPushObject(agent, <0,0,100>, ZERO_VECTOR, FALSE);
                }
                else
                {
                    // owner has mounted
                    llRequestPermissions(agent, PERMISSION_TRIGGER_ANIMATION | PERMISSION_TAKE_CONTROLS);
                    llPlaySound("start", 0.40);
                }
            }
            else
            {
                // dismount
                llStopSound();
                llSetStatus(STATUS_PHYSICS, FALSE);
                llReleaseControls();
                llStopAnimation("motorcycle_sit");
                llPlaySound("off", 0.4);
            }
        }

    }

    run_time_permissions(integer perm)
    {
        if (perm & PERMISSION_TAKE_CONTROLS)
        {
            llTakeControls(CONTROL_FWD | CONTROL_BACK | CONTROL_RIGHT | CONTROL_LEFT 
                           | CONTROL_ROT_RIGHT | CONTROL_ROT_LEFT | CONTROL_UP, TRUE, FALSE);
            llLoopSound("on", 1.0);
            llSetStatus(STATUS_PHYSICS, TRUE);
            // reset the global accumulators
            gAngularMotor = <0, 0, 0>;
            gLinearMotor = <0, 0, 0>;
            gBank = 0.0;
        }
        if (perm & PERMISSION_TRIGGER_ANIMATION)
        {
            llStartAnimation("motorcycle_sit");
        }
    }
    
    control(key id, integer level, integer edge)
    {
        // The idea here is to ramp up the motors when the keys are held down for a long 
        // time and to let the motors decay after they are let go.  This allows fine-
        // tuning of the motion of the vehicle by throttling the key controls.
        //
        // Note that this probably doesn't work well when the client FPS and/or the server
        // FPS is lagging.  So for best results you'll want to turn off as much visual 
        // effects as you can tolerate, and drive in the more empty areas.
        
        // linear
        integer key_control = FALSE;
        if(level & CONTROL_FWD)
        {
            gLinearMotor.x = gLinearMotor.x + gLinearRamp * (gMaxFwdSpeed - gLinearMotor.x);
            key_control = TRUE;
        }
        if(level & CONTROL_BACK)
        {
            gLinearMotor.x = gLinearMotor.x + gLinearRamp * (gMaxBackSpeed - gLinearMotor.x);
            key_control = TRUE;
        }
        if (key_control)
        {
            llSetVehicleVectorParam(VEHICLE_LINEAR_MOTOR_DIRECTION, gLinearMotor);
            key_control = FALSE;
        }
        else
        {
            if (gLinearMotor.x > 15 || gLinearMotor.x < -5)
            {
                // Automatically reduce the motor if keys are let up when moving fast.
                gLinearMotor.x *= 0.8;
                llSetVehicleVectorParam(VEHICLE_LINEAR_MOTOR_DIRECTION, gLinearMotor);
                
            }
            else
            {
                // reduce the linear motor accumulator for the next control() event
                gLinearMotor.x *= 0.8;
            }
                
        }
        
        // angular
        if(level & (CONTROL_RIGHT|CONTROL_ROT_RIGHT))
        {
            gAngularMotor.x = gAngularMotor.x + gAngularRamp * (gMaxTurnSpeed - gAngularMotor.x);
            key_control = TRUE;
        }
        if(level & (CONTROL_LEFT | CONTROL_ROT_LEFT))
        {
            gAngularMotor.x = gAngularMotor.x - gAngularRamp * (gMaxTurnSpeed + gAngularMotor.x);
            key_control = TRUE;
        }
        if(level & CONTROL_UP)
        {
            gAngularMotor.y = gAngularMotor.y - gAngularRamp * (gMaxWheelieSpeed + gAngularMotor.y);
            key_control = TRUE;
        }
        if (key_control)
        {
            // turn on banking and apply angular motor
            gBank = 3.0;
            llSetVehicleFloatParam(VEHICLE_BANKING_EFFICIENCY, gBank);
            llSetVehicleVectorParam(VEHICLE_ANGULAR_MOTOR_DIRECTION, gAngularMotor);
            gAngularMotor *= 0.95;   // light attenuation
        }
        else
        {
            if (gAngularMotor.x > 4
                || gAngularMotor.x < -4)
            {
                // We were turning hard, but no longer  ==> reduce banking to help 
                // the motorcycle travel straight when bouncing on rough terrain.
                // Also, turn off the angular motor ==> faster upright recovery.
                gAngularMotor *= 0.4;
                gBank *= 0.5;
                llSetVehicleFloatParam(VEHICLE_BANKING_EFFICIENCY, gBank);
                llSetVehicleVectorParam(VEHICLE_ANGULAR_MOTOR_DIRECTION, gAngularMotor);
            }
            else
            {
                // reduce banking for straigher travel when not actively turning
                gAngularMotor *= 0.5;
                gBank *= 0.8;
                llSetVehicleFloatParam(VEHICLE_BANKING_EFFICIENCY, gBank);
            }
        }
    }
}
