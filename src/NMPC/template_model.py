#
#   This file is part of do-mpc
#
#   do-mpc: An environment for the easy, modular and efficient implementation of
#        robust nonlinear model predictive control
#
#   Copyright (c) 2014-2018 Sergio Lucia, Alexandru Tatulea-Codrean
#                        TU Dortmund. All rights reserved
#
#   do-mpc is free software: you can redistribute it and/or modify
#   it under the terms of the GNU Lesser General Public License as
#   published by the Free Software Foundation, either version 3
#   of the License, or (at your option) any later version.
#
#   do-mpc is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU Lesser General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with do-mpc.  If not, see <http://www.gnu.org/licenses/>.
#

from casadi import *
import numpy as NP
import core_do_mpc
#import math
def model(waypoint, obstacles,pose,twist,wt):

    """
    --------------------------------------------------------------------------
    template_model: define the non-uncertain parameters
    --------------------------------------------------------------------------
    """
    """
    --------------------------------------------------------------------------
    template_model: define uncertain parameters, states and controls as symbols
    --------------------------------------------------------------------------
    """
    # Define the uncertainties as CasADi symbols

    alpha   = SX.sym("alpha")
    beta    = SX.sym("beta")
    # Define the differential states as CasADi symbols

    x    = SX.sym("x") # Concentration A
    y    = SX.sym("y") # Concentration B
    theta    = SX.sym("theta") # Reactor Temprature
    #T_K    = SX.sym("T_K") # Jacket Temprature

    # Define the algebraic states as CasADi symbols

    # Define the control inputs as CasADi symbols

    u      = SX.sym("u") # Vdot/V_R [h^-1]
    w  = SX.sym("w") #Q_dot second control input

    # Define time-varying parameters that can change at each step of the prediction and at each sampling time of the MPC controller. For example, future weather predictions

    tv_param_1 = SX.sym("tv_param_1")
    tv_param_2 = SX.sym("tv_param_2")

    """
    --------------------------------------------------------------------------
    template_model: define algebraic and differential equations
    --------------------------------------------------------------------------
    """
    # Define the algebraic equations

    #K_1 = beta * K0_ab * exp((-E_A_ab)/((T_R+273.15)))
    #K_2 =  K0_bc * exp((-E_A_bc)/((T_R+273.15)))
    #K_3 = K0_ad * exp((-alpha*E_A_ad)/((T_R+273.15)))

    # Define the differential equations

    dx=u*cos(theta)
    dy=u*sin(theta)
    dtheta=w
    #dC_a = F*(C_A0 - C_a) -K_1*C_a - K_3*(C_a**2)
    #dC_b = -F*C_b + K_1*C_a -K_2*C_b
    #dT_R = ((K_1*C_a*H_R_ab + K_2*C_b*H_R_bc + K_3*(C_a**2)*H_R_ad)/(-Rou*Cp)) + F*(T_in-T_R) +(((K_w*A_R)*(T_K-T_R))/(Rou*Cp*V_R))
    #dT_K = (Q_dot + K_w*A_R*(T_R-T_K))/(m_k*Cp_k)

    # Concatenate differential states, algebraic states, control inputs and right-hand-sides

    _x = vertcat(x, y, theta)

    _u = vertcat(u, w)

    _xdot = vertcat(dx, dy, dtheta)

    _p = vertcat(alpha, beta)
    #_p = []

    _z = []

    _tv_p = vertcat(tv_param_1, tv_param_2)
    #_tv_p = vertcat()

    """
    --------------------------------------------------------------------------
    template_model: initial condition and constraints
    --------------------------------------------------------------------------
    """
    # Initial condition for the states
    #print "pose",pose
    x_0 = pose[0] # This is the initial concentration inside the tank [mol/l]
    y_0 = pose[1] # This is the controlled variable [mol/l]
    theta_0 = pose[2] #[C]
#    T_K_0 = 130.0 #[C]
    x0 = NP.array([x_0, y_0, theta_0])

    # Bounds on the states. Use "inf" for unconstrained states
    x_lb = -10;			x_ub = 10.0
    y_lb = -10;			y_ub = 10.0
    theta_lb = -3.14;			theta_ub = 3.14
    #T_K_lb = 50.0;			T_K_ub = 180
    X_lb = NP.array([x_lb, y_lb, theta_lb])
    X_ub = NP.array([x_ub, y_ub, theta_ub])

    # Bounds on the control inputs. Use "inf" for unconstrained inputs
    u_lb = 0.4;                 u_ub = 0.7;
    w_lb = -1.0;         w_ub = 1.0;
    U_lb = NP.array([u_lb, w_lb])
    U_ub = NP.array([u_ub, w_ub])
    u0 = NP.array(twist)

    # Scaling factors for the states and control inputs. Important if the system is ill-conditioned
    x_scaling = NP.array([1.0, 1.0, 1.0])
    u_scaling = NP.array([1.0, 1.0])

    # Other possibly nonlinear constraints in the form cons(x,u,p) <= cons_ub
    # Define the expresion of the constraint (leave it empty if not necessary)
    cons = vertcat([])
    # Define the lower and upper bounds of the constraint (leave it empty if not necessary)
    cons_ub = NP.array([])

    # Activate if the nonlinear constraints should be implemented as soft constraints
    soft_constraint = 0
    # Penalty term to add in the cost function for the constraints (it should be the same size as cons)
    penalty_term_cons = NP.array([])
    # Maximum violation for the constraints
    maximum_violation = NP.array([0])

    # Define the terminal constraint (leave it empty if not necessary)
    cons_terminal = vertcat()
    # Define the lower and upper bounds of the constraint (leave it empty if not necessary)
    cons_terminal_lb = NP.array([])
    cons_terminal_ub = NP.array([])


    """
    --------------------------------------------------------------------------
    template_model: cost function
    --------------------------------------------------------------------------
    """
    Y_ref=waypoint
    
    # Define the cost function
    # Lagrange term
    #lterm =  1e4*((C_b - 0.9)**2 + (C_a - 1.1)**2)
    #lterm = 10*(x-Y_ref[0])**2+10*(y-Y_ref[1])**2+(theta-Y_ref[2])**2 + (lam)*exp(-(gamma+((xo[0]-x)**2)/(rx**2)+((xo[1]-y)**2)/(ry**2)))
    if len(obstacles.a):
        lam=wt[3]
        gamma=1    
        V = 0 
        for i in xrange(len(obstacles.a)):
            if obstacles.a[i]:
                rx=obstacles.a[i]
                ry=obstacles.b[i]
                phi = atan(obstacles.m[i])
                xo=obstacles.centroid[i]
                

                R = [[cos(theta_0), -sin(theta_0)],[sin(theta_0),cos(theta_0)]]

                xo = np.matmul(R,xo) + pose[0:2]
                #print "obs position",xo
                V = V + (lam)/((gamma+(( (xo[0]-x)*cos(phi) + (xo[1]-y)*sin(phi) )**2)/(rx**2) + (( (xo[0]-x)*sin(phi) - (xo[1]-y)*cos(phi) )**2)/(ry**2) ))


        lterm = wt[0]*(x-Y_ref[0])**2 + \
            wt[1]*(y-Y_ref[1])**2 + \
            wt[2]*(theta-Y_ref[2])**2 + \
            V        
        
    else:
        lterm = wt[0]*(x-Y_ref[0])**2 + \
                wt[1]*(y-Y_ref[1])**2 + \
                wt[2]*(theta-Y_ref[2])**2
            
    #lterm =  - C_b
    # Mayer term
#    mterm =  1e4*((C_b - 0.9)**2 + (C_a - 1.1)**2)
    mterm = 0
    #mterm =  - C_b
    # Penalty term for the control movements
    rterm = NP.array([0.0, 0.0])



    """
    --------------------------------------------------------------------------
    template_model: pass information (not necessary to edit)
    --------------------------------------------------------------------------
    """
    model_dict = {'x':_x,'u': _u, 'rhs':_xdot,'p': _p, 'z':_z,'x0': x0,'x_lb': X_lb,'x_ub': X_ub, 'u0':u0, 'u_lb':U_lb, 'u_ub':U_ub, 'x_scaling':x_scaling, 'u_scaling':u_scaling, 'cons':cons,
    "cons_ub": cons_ub, 'cons_terminal':cons_terminal, 'cons_terminal_lb': cons_terminal_lb,'tv_p':_tv_p, 'cons_terminal_ub':cons_terminal_ub, 'soft_constraint': soft_constraint, 'penalty_term_cons': penalty_term_cons, 'maximum_violation': maximum_violation, 'mterm': mterm,'lterm':lterm, 'rterm':rterm}

    model = core_do_mpc.model(model_dict)

    return model
