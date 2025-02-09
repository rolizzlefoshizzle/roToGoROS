from bagpy import bagreader
import pandas as pd
import matplotlib.pyplot as plt

# user in
cases = ['stayIn', 'long']
numRuns = 50

# pseudocode
# for each method
# - for each run
# - - extract statistics
# - - - individual statistics (pass/fail)
# - - - correlations
# - average the statistics
# print the statistics

methods = ['formSplitRob', 'robustness']
results = [[{}, {}, {}, {}], [{}, {}, {}, {}]]
for k in range(len(cases)):
    for j in range(len(methods)):
        formulaSizeMonitorTimeCorrelation = []
        clockTimePlanTimeCorrelation = []
        robsFinalValue = []
        robsFinalValueSuccesses = []
        satisfaction = []
        for i in range(numRuns):
            b = bagreader(cases[k] + "/" +
                          methods[j] + str(i) + ".bag")
            # exec time vs plan time correlation
            xPropFile = b.message_by_topic('/logExecTime')
            yPropFile = b.message_by_topic('/logPlanTime')
            xPropData = pd.read_csv(xPropFile)
            yPropData = pd.read_csv(yPropFile)
            correlation = xPropData['data'].corr(
                yPropData['data'])
            clockTimePlanTimeCorrelation.append(correlation)
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
            # print(methods[j])
            # print(rob)
            # print(robLow)
            # print(robHigh)
        results[k][j]['clockTimePlanTimeCorrelation'] = clockTimePlanTimeCorrelation
        results[k][j]['robsFinalValue'] = robsFinalValue
        results[k][j]['robsFinalValueSuccesses'] = robsFinalValueSuccesses
        results[k][j]['satisfaction'] = satisfaction
        clockTimePlanTimeCorrelation = sum(
            clockTimePlanTimeCorrelation)/len(clockTimePlanTimeCorrelation)
        robsFinalValue = sum(robsFinalValue)/len(robsFinalValue)
        if len(robsFinalValueSuccesses) > 0:
            robsFinalValueSuccesses = sum(
                robsFinalValueSuccesses)/len(robsFinalValueSuccesses)
        else:
            robsFinalValueSuccesses = -10000
        satisfaction = sum(satisfaction)/len(satisfaction)
        results[k][j]['clockTimePlanTimeCorrelationMean'] = [
            clockTimePlanTimeCorrelation]
        results[k][j]['robsFinalValueMean'] = [robsFinalValue]
        results[k][j]['robsFinalValueSuccessesMean'] = [
            robsFinalValueSuccesses]
        results[k][j]['satisfactionMean'] = [satisfaction]

for k in range(len(cases)):
    for key in results[k][0].keys():
        print("\n\n\nAverage results for " + key)
        for i in range(len(methods)):
            if len(results[k][i][key]) == 1:
                print(methods[i] + ": " + str(results[k][i][key][0]))

fig, ax = plt.subplots(4)
myKeys = ['clockTimePlanTimeCorrelation',
          'robsFinalValue',
          'robsFinalValueSuccesses',
          'satisfaction']
myLabels = ['Execution Time \n Plan Time \n Correlation',
            'Robustness',
            'Robustness\n(Only Successes)',
            'Satisfaction\n Rate']
myMethods = [r'$\phi_{stayIn}-RCPE$', r'$\phi_{stayIn}-Full$',
             r'$\phi_{wearhouse}-RCPE$', r'$\phi_{wearhouse}-Full$']
for j in range(len(myKeys)):
    data = []
    for k in range(len(cases)):
        for i in range(len(methods)):
            data.append(results[k][i][myKeys[j]])
    ax[j].boxplot(data, showmeans=True)
    ax[j].set_ylabel(myLabels[j])
    # ax[j].yaxis.label.set_rotation(45)
    if j == 3:
        ax[j].set_xticklabels(myMethods)
    else:
        ax[j].set_xticklabels([])

plt.show()
