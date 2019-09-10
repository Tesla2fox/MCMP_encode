import plotly.graph_objects as go
import plotly
import numpy as np


class Pnt:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y
    def pnt2dict(self):
        dic = dict(x=x, y=y)
        return dic
    def display(self):
        print('x = ', self.x, 'y = ', self.y)

class Rect:
    def __init__(self, pnt=Pnt(), width=0, height=0):
        self.x0 = pnt.x
        self.y0 = pnt.y
        self.x1 = self.x0 + width
        self.y1 = self.y0 + height

    def goShape(self):
        dic = go.layout.Shape()
        dic['type'] = 'rect'
        dic['x0'] = self.x0
        dic['y0'] = self.y0
        dic['x1'] = self.x1
        dic['y1'] = self.y1
        dic['line'] = dict(color='rgb(128, 0, 128)')
        return dic

    def rect2dict(self):
        dic = dict()
        dic['type'] = 'rect'
        dic['x0'] = self.x0
        dic['y0'] = self.y0
        dic['x1'] = self.x1
        dic['y1'] = self.y1
        dic['line'] = dict(color='rgb(128, 0, 128)')
        return dic



class Env:
    def __init__(self, mat):
        self._mat = mat
        self._shapeLst= []
        self._scatterLst = []
        self.annotations = []

    def addgrid(self):
        g_color = 'blue'
        row = len(self._mat)
        for i in range(row):
            for j in range(len(self._mat[i])):
                pnt = Pnt(i, j)
                rect = Rect(pnt, 1, 1)
                rectDic = rect.rect2dict()
                rectDic['line']['color'] = g_color
                rectDic['line']['width'] = 0.5
                #                rectDic['opacity'] =  1/(int(self.mat[i][j])+1)
                #                rectDic['fillcolor'] = colorLst[int(self.mat[i][j])]
                if (int(self._mat[i][j]) == 1):
                    rectDic['fillcolor'] = 'black'
                #                if(int(self.mat[i][j])==0):
                #                    rectDic['fillcolor'] = colorLst[int(self.mat[i][j])]
                #                getLevelColor(mat[i][j])

                self._shapeLst.append(rectDic)
        # print(len(self.shapeLst))
    def drawPic(self, name='env', fileType=True):
        layout = dict()
        layout['shapes'] = self._shapeLst
        # layout['xaxis'] = dict(showgrid=False)

        layout['xaxis'] = dict(
            autorange=True,
            showgrid=False,
            zeroline=False,
            showline=False,
            # autotick=True,
            ticks='',
            showticklabels=False)

        layout['yaxis'] = dict(
            scaleanchor="x",
            autorange=True,
            showgrid=False,
            zeroline=False,
            showline=False,
            # autotick=True,
            ticks='',
            showticklabels=False)
        layout['xaxis']['range'] = [0, len(self._mat[0])]
        layout['yaxis']['range'] = [0, len(self._mat)]

        # # layout['font'] = dict(
        # #     family='sans-serif',
        # #     sie=25,
        # #     color='#000'
        # # )
        layout['autosize'] = False
        layout['height'] = 1000
        layout['width'] = 1000
        # layout['annotations'] = self.annotations
        #        print(layout)
        fig = go.Figure(data = self._scatterLst, layout = layout)
        # fig.add_trace(self._scatterLst)
        # fig.update_layout(shapes = self._shapeLst)
        # fig.update_layout(layout)
        fig.show()
        # fig = dict(data=self._scatterLst, layout=layout)
        # if (fileType):
        #     plotly.offline.plot(fig, filename=name + '.html', validate=False)
        # else:
        #     pass
            # py.image.save_as(fig, filename=name + '.jpeg')



if __name__ == '__main__':

    row = 20
    col = 20
    robNum = 2
    p = np.array([0.9,0.1])
    np.random.seed(1000)
    rob_x = np.random.randint(20,size = robNum)
    rob_y = np.random.randint(20,size = robNum)
    robPosLst = []
    for i in range(robNum):
        robPosLst.append((rob_x[i],rob_y[i]))
    # 1 means obstacles
    # 0 means way
    print(robPosLst)
    obstacleLst = []
    for rowInd in range(row):
        for colInd in range(col):
            if (rowInd,colInd) in robPosLst:
                obstacleLst.append(0)
            else:
                obstacleLst.append(np.random.choice([0,1],p =p.ravel()))
    _row = row
    _col = col
    _obstacleLst = obstacleLst
    _robNum = len(robPosLst)
    _mat = np.zeros([_row, _col])
    # print(self._mat)
    ind = 0
    for rowInd in range(row):
        for colInd in range(col):
            _mat[rowInd][colInd] = _obstacleLst[ind]
            ind = ind + 1

    env = Env(_mat)
    env.addgrid()
    env.drawPic()
    print('ss')


