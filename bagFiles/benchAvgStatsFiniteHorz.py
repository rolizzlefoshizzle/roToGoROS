from bagpy import bagreader
from statistics import mean
import pandas as pd
import matplotlib.pyplot as plt

# user in
case = 'stayIn'
numRuns = 5

numVia = [1, 2, 3, 4, 5]
horizons = [5, 10, 15, 20]
results = [{}, {}, {}, {}]
hyperResults = [[{}, {}, {}, {}],
                [{}, {}, {}, {}],
                [{}, {}, {}, {}],
                [{}, {}, {}, {}],
                [{}, {}, {}, {}]]

for k in range(len(numVia)):
    for j in range(len(horizons)):
        avgPlanTimes = []
        robsFinalValue = []
        robsFinalValueSuccesses = []
        satisfaction = []
        for i in range(numRuns):
            b = bagreader("finiteHorz/" + case + "/via" + str(numVia[k])
                          + "horz" + str(horizons[j]) + "run"
                          + str(i) + ".bag")
            # xPropFile = b.message_by_topic('/logExecTime')
            yPropFile = b.message_by_topic('/logPlanTime')
            # xPropData = pd.read_csv(xPropFile)
            yPropData = pd.read_csv(yPropFile)

            avgPlanTime = yPropData['data'].mean()
            avgPlanTimes.append(avgPlanTime)
            # robustness
            xPropFile = b.message_by_topic('/rosi')
            xPropData = pd.read_csv(xPropFile)
            rob = xPropData['data_0'].values[-1]
            robLow = xPropData['data_1'].values[-1]
            robHigh = xPropData['data_2'].values[-1]
            if rob > 0:
                sat = 1
                robsFinalValueSuccesses.append(rob)
            else:
                sat = 0
            robsFinalValue.append(rob)
            satisfaction.append(sat)
            # print(horizons[j])
            # print(rob)
            # print(robLow)
            # print(robHigh)
        hyperResults[k][j]['avgPlanTimes'] = avgPlanTimes
        hyperResults[k][j]['robsFinalValue'] = robsFinalValue
        hyperResults[k][j]['robsFinalValueSuccesses'] = robsFinalValueSuccesses
        hyperResults[k][j]['satisfaction'] = satisfaction
        # avgPlanTimes = sum(
        #     avgPlanTimes)/len(avgPlanTimes)
        robsFinalValue = sum(robsFinalValue)/len(robsFinalValue)
        if len(robsFinalValueSuccesses) > 0:
            robsFinalValueSuccesses = sum(
                robsFinalValueSuccesses)/len(robsFinalValueSuccesses)
        else:
            robsFinalValueSuccesses = -10000
        satisfaction = sum(satisfaction)/len(satisfaction)
        hyperResults[k][j]['avgPlanTimes'] = avgPlanTimes
        hyperResults[k][j]['robsFinalValueMean'] = [robsFinalValue]
        hyperResults[k][j]['robsFinalValueSuccessesMean'] = [
            robsFinalValueSuccesses]
        hyperResults[k][j]['satisfactionMean'] = [satisfaction]


for k in range(len(numVia)):
    print("\n\nResults for " + str(numVia[k]) + "via points")
    results = hyperResults[k]
    for key in results[0].keys():
        print("\n\n\nResults for " + key)
        for i in range(len(horizons)):
            print(str(horizons[i]) + ": " + str(results[i][key]))

fig, ax = plt.subplots(4)
myKeys = ['avgPlanTimes',
          'robsFinalValue',
          'robsFinalValueSuccesses',
          'satisfaction']
myLabels = ['avgPlanTimes',
            'Robustness',
            'Robustness\n(Only Successes)',
            'Satisfaction\n Rate']

myHorizons = ['5', '10', '15', '20']

for k in range(len(numVia)):
    results = hyperResults[k]
    for j in range(len(myKeys)):
        print("\n\nBig Debug")
        print(myKeys[j])
        data = []
        for i in range(len(horizons)):
            print("debugIn")
            print(results[i][myKeys[j]])
            myDatList = results[i][myKeys[j]]
            if len(myDatList) > 0:
                data.append(mean(myDatList))
            else:
                data.append(None)
                # data.append(-10000)
        print("debug")
        print(data)
        ax[j].plot(horizons, data, label=str(numVia[k])+" via points")
        ax[j].set_ylabel(myLabels[j])
        # ax[j].yaxis.label.set_rotation(45)
        # if j == 3:
        #     ax[j].set_xticklabels(myHorizons)
        # else:
        #     ax[j].set_xticklabels([])
ax[3].legend()

plt.show()
