from bagpy import bagreader
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import stlrom
import formulaProgressionLib

# user in
case = 'stayIn'
runIndex = 0
# which planner run to use, for robustness comparison plot
comparisonMethod = 'robustness'


def initialize_driver(formulaStr):
    testMonitor = formulaProgressionLib.rcpeManager(
        "signal x, y, xe, ye")
    testMonitor.add_predicate("x[t]>0")
    testMonitor.add_predicate("x[t]>4")
    testMonitor.add_predicate("x[t]<5")
    testMonitor.add_predicate("y[t]>0")
    testMonitor.add_predicate("y[t]>4")
    testMonitor.add_predicate("y[t]<5")
    testMonitor.add_predicate("y[t]<1")
    testMonitor.add_predicate(
        "(((x[t]-xe[t])*(x[t]-xe[t]))+((y[t]-ye[t])*(y[t]-ye[t])))<0.25", "region")

    testMonitor.addSubform(
        "((x[t]>4)&(y[t]>4))&((x[t]<5)&(y[t]<5))", "goal")

    if formulaStr == 'stayIn.stl':
        testMonitor.setFormula(
            "(F[0,10](G[0,5](region)))&(F[15,20](goal))")
        timeHorz = 10.0
    elif formulaStr == 'long.stl':
        testMonitor.setFormula(
            # there was a bug in the RoSI tool where the time domain of a predicate nested inside of a nested eventually was incorrect, leading to a non-monotonically decreasing upper bound, which is incorrect and hurt planning performance. region&region forces a conjunction in there to sidestep the bug.
            "(G[0,80](((!(region))|(F[0,10](goal)))&((!(goal))|(F[0,10]((region)&(region))))))&(F[0,5](goal))")
        timeHorz = 90.0

    return testMonitor


def constructRobustnessOnlineCosts(b):
    # first, make actual trajectory
    observationsFile = b.message_by_topic('/stateObservations')
    observationsData = pd.read_csv(observationsFile).values[:, 3:]

    # next, initialize a monitor
    monitor = initialize_driver(case+".stl")

    # next, reconstruct formula and make driver
    formula = monitor.printDriver()
    driver = stlrom.STLDriver()

    driver.parse_string(formula)

    trajectory = np.hstack(
        (observationsData[:, 0:3], observationsData[:, 5:]))
    # now, iterate through plans
    planFile = b.message_by_topic('/traj')
    planData = pd.read_csv(planFile).values[:, 3:]
    myRobustnessRobs = []
    for i in range(planData.shape[0]):  # for each plan
        plan = np.reshape(
            planData[i], [round(len(planData[i])/7), 7])
        stateSignal = plan[:, 0:3]
        # find index in trajectory plan starting time
        index = np.searchsorted(
            trajectory[:, 0], stateSignal[0, 0])
        # find predicates
        preds = trajectory[index, 3:]

        # find past
        past = trajectory[0:index, :]

        futureSignal = np.hstack((stateSignal, np.tile(
            preds, (stateSignal.shape[0], 1))))

        signal = np.vstack((past, futureSignal))

        driver.data = signal

        robs = driver.get_online_rob("phi")

        myRobustnessRobs.append(-robs[2])

    return np.array(myRobustnessRobs)


def constructFormSplitRobOnlineCosts(b):
    # first, make actual trajectory
    observationsFile = b.message_by_topic('/stateObservations')
    observationsData = pd.read_csv(observationsFile).values[:, 3:]

    # next, initialize a monitor
    splitter = initialize_driver(case+".stl")

    # next, reconstruct formulas
    formulas = []
    for i in range(observationsData.shape[0]):
        obsservation = list(observationsData[i][0:3])
        obsservation.extend(list(observationsData[i][5:]))
        splitter.update_data(obsservation)
        driverString = splitter.printSplit()
        formulas.append(driverString)

    trajectory = np.hstack(
        (observationsData[:, 0:3], observationsData[:, 5:]))
    # now, iterate through plans
    planFile = b.message_by_topic('/traj')
    planData = pd.read_csv(planFile).values[:, 3:]
    memoryValFile = b.message_by_topic('/logMemoryVal')
    memoryValData = pd.read_csv(memoryValFile)
    memoryVal = memoryValData['data'][0]  # how much memory we need
    myFormSplitRobs = []
    for i in range(planData.shape[0]):  # for each plan
        plan = np.reshape(
            planData[i], [round(len(planData[i])/7), 7])
        stateSignal = plan[:, 0:3]
        # find index in trajectory plan starting time
        index = np.searchsorted(
            trajectory[:, 0], stateSignal[0, 0])
        # find index in trajectory where "memory" begins
        index1 = np.searchsorted(
            trajectory[:, 0], stateSignal[0, 0]-memoryVal)-1
        if index1 < 0:
            index1 = 0
        # find predicates
        preds = trajectory[index, 3:]

        # find formula
        formula = formulas[index]

        # find past
        past = trajectory[index1:index, :]

        futureSignal = np.hstack((stateSignal, np.tile(
            preds, (stateSignal.shape[0], 1))))

        signal = np.vstack((past, futureSignal))

        driver = stlrom.STLDriver()

        driver.parse_string(formula)

        driver.data = signal

        robs = driver.get_online_rob("phi")

        myFormSplitRobs.append(-robs[2])

    return np.array(myFormSplitRobs)


# def analyzeTrajectoryAgainstPredicates(b):
#     # first, make actual trajectory
#     observationsFile = b.message_by_topic('/stateObservations')
#     observationsData = pd.read_csv(observationsFile).values[:, 3:]
#     trajectory = np.hstack(
#         (observationsData[:, 0:3], observationsData[:, 5:]))
#
#     # next, initialize a monitor
#     splitter = initialize_splitter(6, case+".stl")
#
#     # create drivers for each of the propositions
#     workspaceDriver = stlrom.STLDriver()
#     workspaceDriver.parse_string(
#         'signal x,y,left,right,bottom,top\n\nphi:=(x[t]>0) and (x[t]<5) and (y[t]>0) and (y[t]<5)')
#     goal1Driver = stlrom.STLDriver()
#     goal1Driver.parse_string(
#         'signal x,y,left,right,bottom,top\n\nphi:=(x[t]>4) and (y[t]>4)')
#     # goal2Driver = stlrom.STLDriver()
#     # goal2Driver.parse_string(
#     #     'signal x,y,left,right,bottom,top\n\nphi:=(x[t]>4) and (y[t]<1)')
#     regionDriver = stlrom.STLDriver()
#     regionDriver.parse_string(
#         'signal x,y,left,right,bottom,top\n\nphi:=(x[t]>left[t]) and (x[t]<right[t]) and (y[t]>bottom[t]) and (y[t]<top[t])')
#
#     myDrivers = [workspaceDriver, goal1Driver, regionDriver]
#
#     # iterate through trajectory, evaluating propositions
#     trajEvals = np.array([]).reshape(0, 3)
#     for i in range(trajectory.shape[0]):  # for each point
#         pointEvals = []
#         for j in range(len(myDrivers)):  # for each driver
#             myDrivers[j].data = np.array([trajectory[i]])
#             robs = myDrivers[j].get_online_rob("phi")
#             if robs[0] > 0:
#                 pointEvals.append(j+1)
#             else:
#                 pointEvals.append(0)
#         trajEvals = np.vstack((trajEvals, np.array(pointEvals)))
#
#     return trajEvals


methods = ['formSplitRob', 'robustness']
functions = [constructFormSplitRobOnlineCosts, constructRobustnessOnlineCosts]
colors = ['g', 'b']
lw = [3, 1]

# # subplot version - this one's uggly
# fig, axs = plt.subplots(4)
# fig.suptitle('rosi vs costs')
# # plot rosi upper
# for i in range(len(methods)):
#     b = bagreader(case + "/" +
#                   methods[i] + str(runIndex) + ".bag")
#     xPropFile = b.message_by_topic('/logManagerTime')
#     xPropData = pd.read_csv(xPropFile)
#     yPropFile = b.message_by_topic('/rosi')
#     yPropData = pd.read_csv(yPropFile)
#     axs[i].plot(xPropData['data'], yPropData['data_2'],
#                 '--' + colors[i], label=methods[i] + r'$[\rho]^{\uparrow}$')
#     xPropFile = b.message_by_topic('/logExecTime')
#     xPropData = pd.read_csv(xPropFile)
#     yPropFile = b.message_by_topic('/logCost')
#     yPropData = pd.read_csv(yPropFile)
#     axs[i].plot(xPropData['data'], -1*yPropData['data'],
#                 'o-' + colors[i], label=methods[i])
#     for j in range(len(methods)):
#         constructedData = functions[j](b)
#         axs[i].plot(xPropData['data'], -1*constructedData,
#                     '*-' + colors[j], label=methods[j])
#     axs[i].legend()
#     axs[i].set_title('a run of ' + methods[i])
#
# fig.supxlabel('execution time')
# fig.supylabel('robustness')

# single form version version - this one's less ugly
plt.figure()
# plot rosi upper
b = bagreader(case + "/" +
              comparisonMethod + str(runIndex) + ".bag")
xPropFile = b.message_by_topic('/logExecTime')
xPropData = pd.read_csv(xPropFile)
for j in range(len(methods)):
    constructedData = functions[j](b)
    plt.plot(xPropData['data'], -1*constructedData,
             colors[j], label=methods[j], linewidth=lw[j])
xPropFile = b.message_by_topic('/logManagerTime')
xPropData = pd.read_csv(xPropFile)
yPropFile = b.message_by_topic('/rosi')
yPropData = pd.read_csv(yPropFile)
plt.plot(xPropData['data'], yPropData['data_2'],
         'r--', label=r'$[\rho]^{\uparrow}$')
plt.legend()
plt.title('A run of ' + case)
plt.xlabel('Execution Time')
plt.ylabel('Robustness')

lw = [3, 2]

plt.figure()
for i in range(len(methods)):
    b = bagreader(case + "/" +
                  methods[i] + str(runIndex) + ".bag")
    xPropFile = b.message_by_topic('/logExecTime')
    xPropData = pd.read_csv(xPropFile)
    yPropFile = b.message_by_topic('/logPlanTime')
    yPropData = pd.read_csv(yPropFile)
    plt.plot(xPropData['data'], yPropData['data'],
             colors[i], label=methods[i], linewidth=lw[i])
plt.legend()
plt.title("Execution Time vs Planning Time")
plt.ylabel('Planning Time')
plt.xlabel("Execution Time")
# plt.yaxis.set_label_position("right")

# for i in range(len(methods)):
#     b = bagreader(case + "/" +
#                   methods[i] + str(runIndex) + ".bag")
#     xPropFile = b.message_by_topic('/logManagerTime')
#     xPropData = pd.read_csv(xPropFile)
#     yPropFile = b.message_by_topic('/logFormulaSize')
#     yPropData = pd.read_csv(yPropFile)
#     axs[1].plot(xPropData['data'], yPropData['data'],
#                 colors[i], label=methods[i], linewidth=lw[i])
# axs[1].legend()
# axs[1].set_title("Execution Time vs Formula Size")
# axs[1].set_ylabel('Formula Size')
# axs[1].yaxis.set_label_position("right")

# # check the form progression's proposition evaluations
# propLabels = ['Workspace', 'Goal', 'Dynamic Region']
# b = bagreader(case + "/" +
#               "formProg" + str(runIndex) + ".bag")
# myEvals = analyzeTrajectoryAgainstPredicates(b)
# xPropFile = b.message_by_topic('/logManagerTime')
# xPropData = pd.read_csv(xPropFile)
# for i in range(3):
#     axs[2].plot(xPropData['data'], myEvals[:, i],
#                 colors[i], linewidth=lw[i], label=propLabels[i])
# axs[2].set_yticks([0, 1, 2, 3])
# axs[2].set_yticklabels(['False', 'Workspace',
#                         'Goal', 'Dynamic Region'])
# axs[2].tick_params(axis='y', rotation=60)
# axs[2].yaxis.set_label_position("right")
# # plt.yticks([0, 1], ['False', 'True'])
# axs[2].legend()
# axs[2].set_title("Form Prog Run Proposition Evaluations")
# axs[2].set_ylabel('Proposition Evaluation')


# plt.figure()
# for i in range(len(methods)):
#     b = bagreader(case + "/" +
#                   methods[i] + str(runIndex) + ".bag")
#     xPropFile = b.message_by_topic('/logFormulaSize')
#     xPropData = pd.read_csv(xPropFile)
#     yPropFile = b.message_by_topic('/logMonitorTime')
#     yPropData = pd.read_csv(yPropFile)
#     plt.plot(xPropData['data'], yPropData['data'],
#              'o'+colors[i], label=methods[i])
# plt.legend()
# plt.title("Formula Size vs Monitor Time")
# plt.xlabel('Formula Size')
# plt.ylabel('Monitor Time')

# plt.figure()
# for i in range(len(methods)):
#     b = bagreader(case + "/" +
#                   methods[i] + str(runIndex) + ".bag")
#     sparseTimeFile = b.message_by_topic('/logManagerTime')
#     sparseTimeData = pd.read_csv(sparseTimeFile)
#     denseTimeFile = b.message_by_topic('/logExecTime')
#     denseTimeData = pd.read_csv(denseTimeFile)
#     xPropFile = b.message_by_topic('/logFormulaSize')
#     xPropData = pd.read_csv(xPropFile)
#     yPropFile = b.message_by_topic('/logPlanTime')
#     yPropData = pd.read_csv(yPropFile)
#     xPropDataDense = np.interp(
#         denseTimeData['data'], sparseTimeData['data'], xPropData['data'])
#     plt.plot(xPropDataDense, yPropData['data'],
#              '.'+colors[i], label=methods[i])
# plt.legend()
# plt.title("Formula Size vs Planning Time")
# plt.xlabel('Formula Size')
# plt.ylabel('Planning Time')

plt.show()
