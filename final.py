from __future__ import print_function
from __future__ import division
import modern_robotics as mr
import numpy as np
import matplotlib.pyplot as plt
import csv
import logging
"""
Global Variables:
results: TrajectoryGeneration converted from a csv file to array
the_file: The array used to show my scene 6 V-REP results
err_#: Each error's array (used for plotting)
err_final: combines all 6 error arrays
plotarray: Creates the x-values for the plotting
r: wheel radius
l: length from chassis center to a wheel
w: width from chassis center to a wheel
X_err_i: integral of the error
i: increment of how many lines of the_file have been published
Tb0: fixed offset from chassis to base of the arm
screw: screw axes for all arm joints
M0e: end effector frame (at home configuration) relative to base of the arm
config_: chassis planar twist
config_6: chassis' 6-dimensional twist
"""
logging.basicConfig(filename="best.log", level=logging.INFO, format = '%(asctime)s %(processName)s %(name)s %(message)s')
results=[]
the_file = []
err_1=[]
err_2=[]
err_3=[]
err_4=[]
err_5=[]
err_6=[]
err_final=[]
plotarray=[]
r=0.07125 #actual radius is 0.0475
l = 0.47/2
w=0.3/2
X_err_i = 0
i=0
Tb0 = np.array([[1, 0, 0, 0.1662],
                [0, 1, 0, 0],
                [0, 0, 1, 0.0026],
                [0, 0, 0, 1]])
screw = np.array([[0, 0, 1, 0, 0.033, 0],
                [0, -1, 0, -0.5076, 0, 0],
                [0, -1, 0, -0.3526, 0, 0],
                [0, -1, 0, -0.2176, 0, 0],
                [0, 0, 1, 0, 0, 0]]).T
M0e = np.array([[1, 0, 0, 0.033],
                [0, 1, 0, 0],
                [0, 0, 1, 0.6546],
                [0, 0, 0, 1]])
config_ = 0.25 * r * np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],
                                [       1,       1,       1,        1],
                                [      -1,       1,      -1,        1] ])
config_6 = np.concatenate((np.zeros((2,4)), config_, np.zeros((1,4))), axis=0)

def main():
    """
    chassis_phi ... thetalist: robot configuration
    Tsei: initial configuration of end-effector in reference trajectory
    Tsci: cube's initial configuration
    Tscf: cube's final configuration
    Tceg: end-effector position relative to cube when grasping
    Tces: end-effector position above the cube before and after grasping
    k: number of reference trajectory configurations per centisecond
    Kp_: proportional gain matrix value
    Ki_: integral gain matrix value
    Kp: proportional gain matrix
    Ki: integral gain matrix
    delta_t: time-step
    """
    logging.info("Generating animation csv file.")
    chassis_phi = np.pi/4
    chassis_x=.25
    chassis_y=.25
    J1 = 0
    J2 = 0
    J3 = 0.2
    J4 = -1.6
    J5 = 0
    W1 = 0
    W2 = 0
    W3 = 0
    W4 = 0

    thetalist = np.array([[chassis_phi, chassis_x, chassis_y, J1, J2, J3, J4, J5,      W1, W2, W3, W4, 0]])

    Tsei = np.array([[0, 0, 1, 0],
                        [0, 1, 0, 0],
                        [-1, 0, 0, 0.5],
                        [0, 0, 0, 1]])
    Tsci = np.array([[1, 0, 0, 1],
                        [0, 1, 0,   0],
                        [0, 0, 1, 0.025],
                        [0, 0, 0,   1]])
    Tscf=np.array([[0, 1, 0, 0],
                        [-1, 0, 0, -1],
                        [0, 0, 1, 0.025],
                        [0, 0, 0, 1]])
    Tceg = np.array([[0, 0, 1, 0], #.04
                        [0, 1, 0, 0],
                        [-1, 0, 0, 0], #-.02
                        [0, 0, 0, 1]])
    Tces = np.array([[0, 0, 1, 0],
                    [0, 1, 0,   0],
                    [-1, 0, 0, 0.1],
                    [0, 0, 0,   1]])

    k = 1

    Kp_ = 5
    Kp = np.array([[Kp_, 0, 0, 0, 0, 0],
                    [0, Kp_, 0, 0, 0, 0],
                    [0, 0, Kp_, 0, 0, 0],
                    [0, 0, 0, Kp_, 0, 0],
                    [0, 0, 0, 0, Kp_, 0],
                    [0, 0, 0, 0, 0, Kp_],])

    Ki_ = 0.2
    Ki = np.array([[Ki_, 0, 0, 0, 0, 0],
                    [0, Ki_, 0, 0, 0, 0],
                    [0, 0, Ki_, 0, 0, 0],
                    [0, 0, 0, Ki_, 0, 0],
                    [0, 0, 0, 0, Ki_, 0],
                    [0, 0, 0, 0, 0, Ki_],])

    delta_t = 0.01
    the_file.append(thetalist[0])
    TrajectoryGenerator(Tsei,Tsci, Tscf, Tceg, Tces,k)

    with open("penultimate.csv") as csvfile:
        reader = csv.reader(csvfile, quoting=csv.QUOTE_NONNUMERIC) # change contents to floats
        for roww in reader: # each row is a list
            results.append(roww)
    eff_csv = np.asarray(results)

    for cell in range(len(eff_csv)-1):
        Xd = np.array([[eff_csv[cell][0], eff_csv[cell][1], eff_csv[cell][2],eff_csv[cell][9]],
                            [eff_csv[cell][3], eff_csv[cell][4], eff_csv[cell][5],eff_csv[cell][10]],
                            [eff_csv[cell][6], eff_csv[cell][7], eff_csv[cell][8],eff_csv[cell][11]],
                            [0, 0, 0, 1]])
        Xd_next = np.array([[eff_csv[cell+1][0], eff_csv[cell+1][1], eff_csv[cell+1][2],eff_csv[cell+1][9]],
                            [eff_csv[cell+1][3], eff_csv[cell+1][4], eff_csv[cell+1][5],eff_csv[cell+1][10]],
                            [eff_csv[cell+1][6], eff_csv[cell+1][7], eff_csv[cell+1][8],eff_csv[cell+1][11]],
                            [0, 0, 0, 1]])
        X, J_arm, J_chassis = jacobianify(chassis_phi, chassis_x, chassis_y, J1, J2, J3, J4, J5)
        command, X_err = FeedbackControl(X, Xd, Xd_next, Kp, Ki, delta_t, J_arm, J_chassis)
        T_cont = np.array([np.append(command[4:],command[:4])])
        grip1 = eff_csv[cell][-1]
        config0 = np.array([the_file[-1]])
        #print(T_cont.shape)
        chassis_phi, chassis_x, chassis_y, J1, J2, J3, J4, J5, W1, W2, W3, W4, grip1 = NextState(config0, T_cont, grip1, delta_t)
        err_1.append(X_err[0])
        err_2.append(X_err[1])
        err_3.append(X_err[2])
        err_4.append(X_err[3])
        err_5.append(X_err[4])
        err_6.append(X_err[5])
        err_final.append([X_err[0], X_err[1], X_err[2], X_err[3], X_err[4], X_err[5]])
        with open('Xerr.csv', 'a') as csvFile:
                writer = csv.writer(csvFile)
                writer.writerow(err_final[-1])
        csvFile.close()

        with open("scene6.csv","w+") as my_csv:
            csvWriter = csv.writer(my_csv,delimiter=',')
            csvWriter.writerows(the_file)

    for point in range (len(the_file)-1):
        plotarray.append(point/100)
    logging.info("Writing error plot data.")
    plt.plot(plotarray, err_1, label='#1')
    plt.plot(plotarray, err_2, label='#2')
    plt.plot(plotarray, err_3, label='#3')
    plt.plot(plotarray, err_4, label='#4')
    plt.plot(plotarray, err_5, label='#5')
    plt.plot(plotarray, err_6, label='#6')
    plt.legend()
    plt.show()
    logging.info("Done.")
def jacobianify(chassis_phi, chassis_x, chassis_y, J1, J2, J3, J4, J5):
    """
    Converts the robot configuration into Jacobian matrices
    chassis_phi ... J5: robot configuration
    X: current actual end-effector configuration
    J_arm: Jacobian of arm
    J_chassis: Jacobian of chassis/base
    """
    arm_config = np.array([J1, J2, J3, J4, J5])
    T0e = mr.FKinBody(M0e, screw, arm_config)
    Tsb = np.array([[np.cos(chassis_phi), -np.sin(chassis_phi), 0, chassis_x],
                    [np.sin(chassis_phi),  np.cos(chassis_phi), 0, chassis_y],
                    [0,                                      0, 1, 0.0963],
                    [0,                                      0, 0, 1]])
    Ts0 = np.matmul(Tsb, Tb0) #np.dot(Tsb, Tb0)
    X = np.matmul(Ts0, T0e) #np.dot(Ts0, T0e)
    J_arm = mr.JacobianBody(screw, arm_config)
    #Jc_pre = np.linalg.inv(T0e),Tb0
    #J_chassis = np.dot(mr.Adjoint(np.dot(np.linalg.inv(T0e),Tb0)), config_6)
    J_chassis = np.matmul(mr.Adjoint(np.linalg.inv(np.matmul(Tb0,T0e))), config_6)
    return X, J_arm, J_chassis


def NextState(thetalist, dthetalist, grip2, delta_t):
    """
    Calculates upcoming robot configuration
    thetalist:robot configuration
    dthetalist: velocity of each robot joint/wheel
    grip2: gripper status (0 or 1)
    delta_t: time-step
    i: increment of how many states have been calculated
    chassis__phi ... W4__: new configuration
    """
    global i
    chassis_phi, chassis_x, chassis_y, J1, J2, J3, J4, J5, W1, W2, W3, W4, grip3 = thetalist[0]

    #Sets a maximum speed

    for thing in range(len(dthetalist[0])):
        if dthetalist[0][thing] > 9:
            dthetalist[0][thing] = 9

    norm = np.linalg.norm(dthetalist[0])
    """
    if norm > 15:
        dthetalist[0] = dthetalist[0] / norm * 15
    """
    J1_, J2_, J3_, J4_, J5_, W1_, W2_, W3_, W4_ = dthetalist[0]
    Ts_b = np.array([[np.cos(chassis_phi), -np.sin(chassis_phi), 0, chassis_x],
                    [np.sin(chassis_phi),    np.cos(chassis_phi), 0, chassis_y],
                    [0,                                      0, 1, 0.0963],
                    [0,                                      0, 0, 1]])
    i+= 1
    print(i)
    print(" of 2399 (If 2399 is exceeded: stop, delete generated files and try again. Won't happen again)")
    d_th = np.array([[W1_], [W2_], [W3_], [W4_]])*delta_t
    Vb = np.matmul(config_,d_th)#config_.dot(d_th)

    Vb_6 = np.array([[0, 0, Vb[0,0], Vb[1,0], Vb[2,0], 0]]).T
    Tb_b_pre = mr.VecTose3(Vb_6.reshape(-1))
    Tb_b = mr.MatrixExp6(Tb_b_pre)
    Tb = np.matmul(Ts_b, Tb_b) #Ts_b.dot(Tb_b)
    Tb00 = Tb[0][0]
    Tb03 = Tb[0][3]
    Tb13 = Tb[1][3]

    #if type(Tb00) == np.ndarray:
    #    Tb00 = Tb00.astype(float)
    #    Tb03 = (Tb[0][3]).astype(float)
    #    Tb13 = (Tb[1][3]).astype(float)

    chassis__phi = np.arccos(Tb00)
    chassis__x = Tb03
    chassis__y = Tb13
    J1__ = J1 + J1_ * delta_t
    J2__ = J2 + J2_ * delta_t
    J3__ = J3 + J3_ * delta_t
    J4__ = J4 + J4_ * delta_t
    J5__ = J5 + J5_ * delta_t
    W1__ = W1 + W1_ * delta_t
    W2__ = W2 + W2_ * delta_t
    W3__ = W3 + W3_ * delta_t
    W4__ = W4 + W4_ * delta_t

    the_file.append([chassis__phi, chassis__x, chassis__y, J1__, J2__, J3__, J4__, J5__, W1__, W2__, W3__, W4__, grip2])
    return chassis__phi, chassis__x, chassis__y, J1__, J2__, J3__, J4__, J5__, W1__, W2__, W3__, W4__, grip2


def FeedbackControl (X, Xd, Xd_next, Kp, Ki, delta_t, J_arm, J_chassis):
    """
    X_err_i: integral of the error
    V: feed-forward + PI controller equation
    Je: Jacobian of end-effector
    command: end-effector inverse applied to PI equation
    X_err: end-effector error
    """
    global X_err_i

    Vd = mr.se3ToVec(mr.MatrixLog6(np.matmul(mr.TransInv(Xd), Xd_next)))/delta_t
    X_err = mr.se3ToVec(mr.MatrixLog6(np.matmul(mr.TransInv(X), Xd)))
    X_err_i = X_err_i + X_err * delta_t
    a = mr.Adjoint(np.matmul(mr.TransInv(X), Xd)).dot(Vd)
    b = Kp.dot(X_err)
    c = Ki.dot(X_err_i)
    V = a + b + c

    Je = np.concatenate((J_chassis, J_arm), axis=1)
    Je_inv = np.linalg.pinv(Je, 1e-2)
    command = np.matmul(Je_inv,V)

    return command, X_err


def TrajectoryGenerator(Xstart, Tsci, Tscf, Tceg, Tces, k):
    """
    Generates end-effector trajectory between 9 points (8 transitions) and adds them to a csv file
    The trajectory from the first to second standoff position is a screw instead of a Cartesian.
    Xstart: initial configuration of end-effector in reference trajectory (Tsei)
    Tsci: cube's initial configuration
    Tscf: cube's final configuration
    Tceg: end-effector position relative to cube when grasping
    Tces: end-effector position above the cube before and after grasping
    k: number of reference trajectory configurations per centisecond
    """
    Tf = 3

    method = 5
    Xend = np.matmul(Tsci, Tces) #first standoff
    FSL = np.matmul(Tsci, Tceg) #first standoff lowered
    SS = np.matmul(Tscf, Tces) #second standoff
    SSL = np.matmul(Tscf, Tceg) #second standoff lowered
    Xtheend = np.matmul(Tscf, Tces)
    N=Tf*100*k
    N = int(N)
    timegap = Tf / (N - 1.0)
    traj = [[None]] * N
    Rstart, pstart = mr.TransToRp(Xstart)
    Rend, pend = mr.TransToRp(Xend)
    for i in range(N):
        if method == 3:
            s = mr.CubicTimeScaling(Tf, timegap * i)
        else:
            s = mr.QuinticTimeScaling(Tf, timegap * i)
        traj[i] \
        = np.r_[np.c_[np.dot(Rstart, \
        mr.MatrixExp3(mr.MatrixLog3(np.dot(np.array(Rstart).T,Rend)) * s)), \
                   s * np.array(pend) + (1 - s) * np.array(pstart)], \
                   [[0, 0, 0, 1]]]

        row=[]
        for p in range (len(traj[i])-1):
            for l in range (0, 3):
                row.append(traj[i][p][l])
        for m in range (len(traj[i])-1):
            for n in range (0, 4):
                if n == 3:
                    row.append(traj[i][m][n])
        row.append(0)
        with open('penultimate.csv', 'a') as csvFile:
                writer = csv.writer(csvFile)
                writer.writerow(row)
        csvFile.close()


    N = int(N)
    timegap = Tf / (N - 1.0)
    traj = [[None]] * N
    RstartFSL, pstartFSL = mr.TransToRp(Xend)
    RendFSL, pendFSL = mr.TransToRp(FSL)
    for i in range(N):
        if method == 3:
            s = mr.CubicTimeScaling(Tf, timegap * i)
        else:
            s = mr.QuinticTimeScaling(Tf, timegap * i)
        traj[i] \
        = np.r_[np.c_[np.dot(RstartFSL, \
        mr.MatrixExp3(mr.MatrixLog3(np.dot(np.array(RstartFSL).T,RendFSL)) * s)), \
                   s * np.array(pendFSL) + (1 - s) * np.array(pstartFSL)], \
                   [[0, 0, 0, 1]]]
        row=[]
        for q in range (len(traj[i])-1):
            for r in range (0, 3):
                row.append(traj[i][q][r])
        for s in range (len(traj[i])-1):
            for t in range (0, 4):
                if t == 3:
                    row.append(traj[i][s][t])
        row.append(0)
        with open('penultimate.csv', 'a') as csvFile:
                writer = csv.writer(csvFile)
                writer.writerow(row)
        csvFile.close()


    N = int(N)
    timegap = Tf / (N - 1.0)
    traj = [[None]] * N
    RstartFC, pstartFC = mr.TransToRp(FSL) #FC = FIRST CLOSE
    RendFC, pendFC = mr.TransToRp(FSL)
    for i in range(N):
        if method == 3:
            s = mr.CubicTimeScaling(Tf, timegap * i)
        else:
            s = mr.QuinticTimeScaling(Tf, timegap * i)
        traj[i] \
        = np.r_[np.c_[np.dot(RstartFC, \
        mr.MatrixExp3(mr.MatrixLog3(np.dot(np.array(RstartFC).T,RendFC)) * s)), \
                   s * np.array(pendFC) + (1 - s) * np.array(pstartFC)], \
                   [[0, 0, 0, 1]]]
        row=[]
        for u in range (len(traj[i])-1):
            for v in range (0, 3):
                row.append(traj[i][u][v])
        for w in range (len(traj[i])-1):
            for x in range (0, 4):
                if x == 3:
                    row.append(traj[i][w][x])
        row.append(1)
        with open('penultimate.csv', 'a') as csvFile:
                writer = csv.writer(csvFile)
                writer.writerow(row)
        csvFile.close()


    N = int(N)
    timegap = Tf / (N - 1.0)
    traj = [[None]] * N
    RstartFrise, pstartFrise = mr.TransToRp(FSL) #Frise = first rise
    RendFrise, pendFrise = mr.TransToRp(Xend)
    for i in range(N):
        if method == 3:
            s = mr.CubicTimeScaling(Tf, timegap * i)
        else:
            s = mr.QuinticTimeScaling(Tf, timegap * i)
        traj[i] \
        = np.r_[np.c_[np.dot(RstartFrise, \
        mr.MatrixExp3(mr.MatrixLog3(np.dot(np.array(RstartFrise).T,RendFrise)) * s)), \
                   s * np.array(pendFrise) + (1 - s) * np.array(pstartFrise)], \
                   [[0, 0, 0, 1]]]
        row=[]
        for y in range (len(traj[i])-1):
            for z in range (0, 3):
                row.append(traj[i][y][z])
        for aa in range (len(traj[i])-1):
            for ab in range (0, 4):
                if ab == 3:
                    row.append(traj[i][aa][ab])
        row.append(1)
        with open('penultimate.csv', 'a') as csvFile:
                writer = csv.writer(csvFile)
                writer.writerow(row)
        csvFile.close()

    """
    N = int(N)
    timegap = Tf / (N - 1.0)
    traj = [[None]] * N
    RstartSS, pstartSS = mr.TransToRp(Xend) #Frise = first rise
    RendSS, pendSS = mr.TransToRp(SS)
    for i in range(N):
        if method == 3:
            s = mr.CubicTimeScaling(Tf, timegap * i)
        else:
            s = mr.QuinticTimeScaling(Tf, timegap * i)
        traj[i] \
        = np.r_[np.c_[np.dot(RstartSS, \
        mr.MatrixExp3(mr.MatrixLog3(np.dot(np.array(RstartSS).T,RendSS)) * s)), \
                   s * np.array(pendSS) + (1 - s) * np.array(pstartSS)], \
                   [[0, 0, 0, 1]]]
    """
    N = int(N)
    timegap = Tf / (N - 1.0)
    traj = [[None]] * N
    for i in range(N):
        if method == 3:
            s = mr.CubicTimeScaling(Tf, timegap * i)
        else:
            s = mr.QuinticTimeScaling(Tf, timegap * i)
        traj[i] \
        = np.dot(Xend, mr.MatrixExp6(mr.MatrixLog6(np.dot(mr.TransInv(Xend), \
                                                      SS)) * s))

        row=[]
        for ac in range (len(traj[i])-1):
            for ad in range (0, 3):
                row.append(traj[i][ac][ad])
        for ae in range (len(traj[i])-1):
            for af in range (0, 4):
                if af == 3:
                    row.append(traj[i][ae][af])
        row.append(1)
        with open('penultimate.csv', 'a') as csvFile:
                writer = csv.writer(csvFile)
                writer.writerow(row)
        csvFile.close()

    N = int(N)
    timegap = Tf / (N - 1.0)
    traj = [[None]] * N
    RstartSSL, pstartSSL = mr.TransToRp(SS) #Frise = first rise
    RendSSL, pendSSL = mr.TransToRp(SSL)
    for i in range(N):
        if method == 3:
            s = mr.CubicTimeScaling(Tf, timegap * i)
        else:
            s = mr.QuinticTimeScaling(Tf, timegap * i)
        traj[i] \
        = np.r_[np.c_[np.dot(RstartSSL, \
        mr.MatrixExp3(mr.MatrixLog3(np.dot(np.array(RstartSSL).T,RendSSL)) * s)), \
                   s * np.array(pendSSL) + (1 - s) * np.array(pstartSSL)], \
                   [[0, 0, 0, 1]]]
        row=[]
        for ag in range (len(traj[i])-1):
            for ah in range (0, 3):
                row.append(traj[i][ag][ah])
        for ai in range (len(traj[i])-1):
            for aj in range (0, 4):
                if aj == 3:
                    row.append(traj[i][ai][aj])
        row.append(1)
        with open('penultimate.csv', 'a') as csvFile:
                writer = csv.writer(csvFile)
                writer.writerow(row)
        csvFile.close()


    N = int(N)
    timegap = Tf / (N - 1.0)
    traj = [[None]] * N
    RstartSSU, pstartSSU = mr.TransToRp(SSL) #Frise = first rise
    RendSSU, pendSSU = mr.TransToRp(SSL)
    for i in range(N):
        if method == 3:
            s = mr.CubicTimeScaling(Tf, timegap * i)
        else:
            s = mr.QuinticTimeScaling(Tf, timegap * i)
        traj[i] \
        = np.r_[np.c_[np.dot(RstartSSU, \
        mr.MatrixExp3(mr.MatrixLog3(np.dot(np.array(RstartSSU).T,RendSSU)) * s)), \
                   s * np.array(pendSSU) + (1 - s) * np.array(pstartSSU)], \
                   [[0, 0, 0, 1]]]
        row=[]
        for ak in range (len(traj[i])-1):
            for al in range (0, 3):
                row.append(traj[i][ak][al])
        for am in range (len(traj[i])-1):
            for an in range (0, 4):
                if an == 3:
                    row.append(traj[i][am][an])
        row.append(0)
        with open('penultimate.csv', 'a') as csvFile:
                writer = csv.writer(csvFile)
                writer.writerow(row)
        csvFile.close()


    N = int(N)
    timegap = Tf / (N - 1.0)
    traj = [[None]] * N
    RstartSSD, pstartSSD = mr.TransToRp(SSL) #Frise = first rise
    RendSSD, pendSSD = mr.TransToRp(Xtheend)
    for i in range(N):
        if method == 3:
            s = mr.CubicTimeScaling(Tf, timegap * i)
        else:
            s = mr.QuinticTimeScaling(Tf, timegap * i)
        traj[i] \
        = np.r_[np.c_[np.dot(RstartSSD, \
        mr.MatrixExp3(mr.MatrixLog3(np.dot(np.array(RstartSSD).T,RendSSD)) * s)), \
                   s * np.array(pendSSD) + (1 - s) * np.array(pstartSSD)], \
                   [[0, 0, 0, 1]]]
        row=[]
        for ao in range (len(traj[i])-1):
            for ap in range (0, 3):
                row.append(traj[i][ao][ap])
        for aq in range (len(traj[i])-1):
            for ar in range (0, 4):
                if ar == 3:
                    row.append(traj[i][aq][ar])
        row.append(0)
        with open('penultimate.csv', 'a') as csvFile:
                writer = csv.writer(csvFile)
                writer.writerow(row)
        csvFile.close()
    return traj #maybe no return statement needed

if __name__ == '__main__':
    """
    Runs the main function
    """
    main()
