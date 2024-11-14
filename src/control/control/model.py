from do_mpc.model import Model

def get_model() -> Model:
    # +-+-+-+-+-+-+- MODEL DEFINITION +-+-+-+-+-+-+-

    model_type = 'discrete' # either 'discrete' or 'continuous'
    model = Model(model_type)

    x = model.set_variable(var_type='states', var_name='x', shape=(1,1))
    y = model.set_variable(var_type='states', var_name='y', shape=(1,1))
    orientation = model.set_variable(var_type='states', var_name='orientation', shape=(1,1))

    vx = model.set_variable(var_type='inputs', var_name='vx', shape=(1,1))
    vy = model.set_variable(var_type='inputs', var_name='vy', shape=(1,1))
    angular_velocity = model.set_variable(var_type='inputs', var_name='angular_velocity', shape=(1,1))

    ref_x = model.set_variable(var_type='_tvp', var_name='ref_x', shape=(1,1))
    ref_y = model.set_variable(var_type='_tvp', var_name='ref_y', shape=(1,1))
    ref_orientation = model.set_variable(var_type='_tvp', var_name='ref_orientation', shape=(1,1))

    # +-+-+-+-+-+-+- RIGHT HAND SIDE EQUATION +-+-+-+-+-+-+-

    model.set_rhs('x', x + vx)
    model.set_rhs('y', y + vy)
    model.set_rhs('orientation', orientation + angular_velocity)

    model.setup()

    return model