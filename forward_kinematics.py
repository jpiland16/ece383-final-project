import numpy as np
pi = np.pi
cos = np.cos
sin = np.sin

def get_dh_table(q):
    return np.array([
        [q[0],  0.16250,      0     ,   pi/2 ],
        [q[1],  0      ,     -0.425 ,    0   ],
        [q[2],  0      ,     -0.3922,    0   ],
        [q[3],  0.13330,      0     ,   pi/2 ],
        [q[4],  0.0997 ,      0     ,  -pi/2 ],
        [q[5],  0.0996 ,      0     ,    0   ]
    ])

def dh_to_A_matrix(theta, d, a, alpha):
    """
    Creates an `A` matrix from a single row of a DH table.
    """
    return np.array([
         [cos(theta), -cos(alpha)*sin(theta),  sin(alpha)*sin(theta), a*cos(theta)],
         [sin(theta),  cos(alpha)*cos(theta), -sin(alpha)*cos(theta), a*sin(theta)],
         [0         ,  sin(alpha)           ,  cos(alpha)           , d           ],
         [0         ,  0                    ,  0                    , 1]])

def get_joint_pos(q, joint_id = None):
    """
    Retrieves the position of a specific joint. If joint_id == None, returns
    the position of the end effector.
    """

    dh_table = get_dh_table(q)

    # NOTE: joint ID is 1 for getting the position of the end of the first joint
    if joint_id is None:
        joint_id = len(dh_table)

    A = np.identity(4)

    for i, row in enumerate(dh_table):
        # Continue multiplying by A matrices until the desired joint is reached
        if i == joint_id:
            break
        A = A @ dh_to_A_matrix(*row)

    return A

def get_all_joint_positions(q):
    """
    Create a list of T matrices where 

        Tn = A_1 * A_2 * ... * A_n

    such that the position of each joint can be determined.
    """
    dh_table = get_dh_table(q)

    A = np.identity(4)
    ret = []
    for i, row in enumerate(dh_table):
        ret.append(A[:, :]) # append copy of A to ret
        A = A @ dh_to_A_matrix(*row)

    return ret

def get_ee_pos(q):
    """
    Retrieve the position of the end effector.
    """
    return get_joint_pos(q)

