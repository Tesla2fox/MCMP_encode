import plotly.graph_objects as go
import plotly
import numpy as np

data_g = []
data_fitness = []
figData = []
with open('.//debugData//GA_test_200.dat')  as txtData:
    lines = txtData.readlines()
    for line in lines:
        lineData = line.split()
        if (len(lineData) == 0):
            continue
        else:
            try:
                g = int(lineData[0])
            except Exception as e:
                break
                # pass
            data_g.append(g)
            data_fitness.append(float(lineData[1]))

trace = go.Scatter(mode = 'lines+markers', x = data_g, y = data_fitness, name= 'xx')
figData.append(trace)
layout = dict()
layout['xaxis'] = dict(title='gen')
layout['yaxis'] = dict(title='makespan')
fig = go.Figure(data=figData, layout=layout)
plotly.offline.plot(fig, filename='.//debugData//'+'xxx22')
