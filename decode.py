
import numpy as np
import instance
from enum import  Enum
import drawPath
import Astar as A

# class Pattern(Enum):


class Point:
    """
    表示一个点
    """
    def __init__(self, x, y):
        self.x = x;
        self.y = y

    def __eq__(self, other):
        if self.x == other.x and self.y == other.y:
            return True
        return False

    def __str__(self):
        return "x:" + str(self.x) + ",y:" + str(self.y)


class LawnMainDir(Enum):
    right = 1
    top  = 2
    left = 3
    bottom = 4

class ClockDir(Enum):
    clockwise = -1
    antiClockwise = 1


class RobMode(Enum):
    LAWN = 1
    ASTAR = 2
    STOP = 3
    FARASTAR = 4

def dirCal(lawn_main_dir,clock_dir):
    pass


# mainDir = LawnMainDir.right
# firstDir = LawnMainDir.top
# dic = dict()
# dic[LawnMainDir.right] = [LawnMainDir.top, LawnMainDir.right, LawnMainDir.bottom]
#
#
# mainDir ==
#
#
# def getNextPos()
# if mainDir == LawnMainDir.right:
#     robDir = firstDir
# print(dic)
# if mainDir == LawnMainDir.right:
#     pass
# print('val',LawnMainDir.left.value)
# print('ssss',LawnMainDir.value)



class MCMPDecode(object):
    def __init__(self,ins : instance.MCMPInstance):
        self._pathLst = []

        self._row = ins._row
        self._col = ins._col
        self._ins = ins
        self._mat = ins._mat
        self._robNum = ins._robNum
        self._robPosLst = ins._robPosLst
        self._robRowLst = ins._robRowLst
        self._robColLst = ins._robColLst
        self._actSeq = []
        self._robReachRowLst = ins._robReachRowLst
        self._robReachColLst = ins._robReachColLst
        self._robReachSet = set()
        self._robState = []
        self._robPosLst = []
        self._c_robPosLst = []


        self._parternLst = []
        self._parternLst.append([LawnMainDir.right,LawnMainDir.top])
        self._parternLst.append([LawnMainDir.right,LawnMainDir.bottom])

        self._parternLst.append([LawnMainDir.left,LawnMainDir.top])
        self._parternLst.append([LawnMainDir.left,LawnMainDir.bottom])

        self._parternLst.append([LawnMainDir.top,LawnMainDir.left])
        self._parternLst.append([LawnMainDir.top,LawnMainDir.right])


        self._parternLst.append([LawnMainDir.bottom,LawnMainDir.left])
        self._parternLst.append([LawnMainDir.bottom,LawnMainDir.right])


        self._dirDic = dict()
        self._dirDic[LawnMainDir.right] = [LawnMainDir.top, LawnMainDir.right, LawnMainDir.bottom]
        self._dirDic[LawnMainDir.left] = [LawnMainDir.top, LawnMainDir.left, LawnMainDir.bottom]
        self._dirDic[LawnMainDir.top] = [LawnMainDir.left, LawnMainDir.top, LawnMainDir.right]
        self._dirDic[LawnMainDir.bottom] = [LawnMainDir.left, LawnMainDir.bottom, LawnMainDir.right]

        pass

    def getNextLawnPos(self,robPos,dir):
        resPos = [-1,-1]
        if dir == LawnMainDir.right:
            if robPos[0] < self._col - 1:
               resPos[0] = robPos[0] + 1
               resPos[1] = robPos[1]
        if dir == LawnMainDir.left:
            if robPos[0] > 0:
               resPos[0] = robPos[0] - 1
               resPos[1] = robPos[1]
        if dir == LawnMainDir.top:
            if robPos[1] < self._row -1:
               resPos[0] = robPos[0]
               resPos[1] = robPos[1] + 1
        if dir == LawnMainDir.bottom:
            if robPos[1] > 0:
               resPos[0] = robPos[0]
               resPos[1] = robPos[1] - 1
        return resPos

    def getNextDirPos(self,main_dir,c_dir,p_dir):
        dirSeq = self._dirDic[main_dir]
        c_ind = dirSeq.index(c_dir)
        if c_ind  ==  1:
            if dirSeq.index(p_dir) == 2:
                return dirSeq[0]
            else:
                return dirSeq[2]
        else:
            return dirSeq[1]


    def getWholeLawnPos(self,robID,mainDir,firstDir,stepNum):
        resPathLst = []
        c_Pos = self._c_robPosLst[robID]
        c_robDir = firstDir
        p_robDir = firstDir
        '''
        终止条件
        '''
        for step in range(stepNum):
            pos = self.getNextLawnPos(c_Pos,c_robDir)
            if pos == [-1,-1]:
                p_robDir = c_robDir
                c_robDir = self.getNextDirPos(mainDir,c_robDir,p_robDir)
                if c_robDir == mainDir:
                    pos = self.getNextLawnPos(c_Pos, c_robDir)
                    if pos == [-1,-1]:
                        break
                    else:
                        resPathLst.append(pos)
                        c_Pos = pos
                        c_robDir = self.getNextDirPos(mainDir, c_robDir,p_robDir)
            else:
                resPathLst.append(pos)
                c_Pos = pos
        return resPathLst

    def decode(self,chrom):

        self._coveredGrid = []

        # for robID in range(self._robNum):
        #     dic = dict(step = 1,patternStep = 0, patternSeq = 0,act = True,pos = (self._robRowLst[robID],self._robColLst[robID]))
        #     self._robState.append(dic)
        #     self._pathLst.append([])
        #     self._pathLst[robID].append(dic['pos'])
        #     self._coveredGrid.append(dic['pos'])
        self._robState.clear()
        self._predictPathLst = []

        self._NextState = [[] for x in range(self._robNum)]
        chromCircleTime = 0
        encodeEnd = False
        while True:
            for robID in range(self._robNum):
                if chromCircleTime * robNum * 4  + robID *4 + robNum *2 >= len(chrom):
                    encodeEnd = True
                    break
                else:
                    dic = dict()
                    dic['row'] = chrom[chromCircleTime * robNum * 4  + robNum *2 ]
                    dic['col'] = chrom[chromCircleTime * robNum * 4  + robNum *2  + 1]
                    dic['patternType'] = chrom[chromCircleTime * robNum * 4  + robNum *2  + 2]
                    dic['step'] = chrom[chromCircleTime * robNum * 4  + robNum *2  + 3]
                    self._NextState[robID].append(dic)
            if encodeEnd:
                break
            chromCircleTime  += 1

        print(self._NextState)

        # exit()

        for robID in range(self._robNum):
            dic = dict()
            dic['patternType'] = chrom[2*robID]
            dic['step'] = chrom[2*robID + 1]
            dic['actPatternStep'] = 0
            dic['mode']  = RobMode.LAWN
            dic['farStep'] = 0
            self._robState.append(dic)
            main_dir, first_dir = self._parternLst[dic['patternType']]
            self._c_robPosLst.append((self._robRowLst[robID],self._robColLst[robID]))
            predictPath = self.getWholeLawnPos(robID,main_dir,first_dir,dic['step'])
            print('robID',robID,' mainDir = ', main_dir,' first_dir = ',first_dir)

            self._predictPathLst.append(predictPath)
            self._pathLst.append([self._c_robPosLst[robID]])

        circleTime = 0
        while True:
            circleTime += 1
            for robID in range(self._robNum):
                state = self._robState[robID]
                c_mode = state['mode']
                if c_mode == RobMode.LAWN:
                    act_patternStep = state['actPatternStep']
                    act_patternStep,pathLst = self.getPredictPath(robID,act_patternStep)
                    state['actPatternStep'] = act_patternStep

                    if len(pathLst) == 1:
                        self._pathLst[robID].append(pathLst[0])
                        self._c_robPosLst[robID] = pathLst[0]
                    elif len(pathLst) == 0:
                        farStep = state['farStep']

                        print('robID ', robID)
                        print('farStep ',farStep)

                        while True:
                            if state['farStep'] == len(self._NextState[robID]):
                                state['mode'] = RobMode.STOP
                                break

                            t_row = self._NextState[robID][farStep]['row']
                            t_col = self._NextState[robID][farStep]['col']
                            if self._mat[t_row][t_col] == 1:
                                state['farStep'] += 1
                                continue
                                # state['farStep'] += 1

                            farPathLst = self.farAstar(self._c_robPosLst[robID][0],self._c_robPosLst[robID][1],t_row,t_col)


                            if True:
                                pass
                            # if farPathLst
                            state['mode'] = RobMode.FARASTAR
                            self._pathLst[robID].append(farPathLst[0])
                            self._c_robPosLst[robID] = farPathLst[0]
                            state['far_astarStep'] = 1
                            state['far_astarPath'] = farPathLst
                            state['actPatternStep'] = 0

                            state['patternType'] = self._NextState[robID][farStep]['patternType']
                            state['step'] = self._NextState[robID][farStep]['step']

                            state['farStep'] += 1
                            break

                        # break
                    else:
                        state['mode'] = RobMode.ASTAR
                        self._pathLst[robID].append(pathLst[0])
                        self._c_robPosLst[robID] = pathLst[0]
                        state['astarStep'] = 1
                        state['astarPath'] = pathLst
                if c_mode  == RobMode.ASTAR:
                    astarStep = state['astarStep']
                    self._pathLst[robID].append(state['astarPath'][astarStep])
                    self._c_robPosLst[robID] = state['astarPath'][astarStep]
                    state['astarStep'] += 1
                    if state['astarStep'] == len(state['astarPath']):
                        state['mode'] = RobMode.LAWN
                if c_mode == RobMode.FARASTAR:
                    far_astarStep = state['far_astarStep']
                    self._pathLst[robID].append(state['far_astarPath'][far_astarStep])
                    self._c_robPosLst[robID] = state['far_astarPath'][far_astarStep]
                    state['far_astarStep'] += 1
                    if state['far_astarStep'] == len(state['far_astarPath']):
                        state['mode'] = RobMode.LAWN

                        main_dir, first_dir = self._parternLst[dic['patternType']]
                        # self._c_robPosLst.append((self._robRowLst[robID], self._robColLst[robID]))
                        predictPath = self.getWholeLawnPos(robID, main_dir, first_dir, dic['step'])
                        print('robID', robID, ' mainDir = ', main_dir, ' first_dir = ', first_dir)
                        self._predictPathLst[robID] = predictPath

                    # break
                    # raise Exception("asdsa")
                        # pass
                        # _c_robPosLst
                # getPredictPath(robID,)
            if circleTime > 1000:
                break

        self.drawPath()
        # main_dir,first_dir = self._parternLst[2]
        # _path = self.getWholeLawnPos(0,main_dir,first_dir,455)

    def getPredictPath(self,robID,act_patternStep):
        pathList = []
        c_row,c_col  = self._c_robPosLst[robID]
        if len(self._predictPathLst[robID]) == 0:
            return act_patternStep, pathList
        while True:
            pre_row,pre_col = self._predictPathLst[robID][act_patternStep]
            act_patternStep += 1
            if act_patternStep >= len(self._predictPathLst[robID]):
                return act_patternStep,pathList
            if self._mat[pre_row][pre_col] == 0:
                row_bias = abs(pre_row - c_row)
                col_bias = abs(pre_col - c_col)
                if (row_bias + col_bias) == 1:
                    pathList.append((pre_row,pre_col))
                    return act_patternStep,pathList
                    pass
                else:
                    map2d = self._mat
                    aStar = A.AStar(map2d,Point(c_row,c_col),Point(pre_row,pre_col))
                    a_path = aStar.start()
                    for unit in a_path:
                        pathList.append((unit.x,unit.y))
                    # print(pathList)
                    # raise  Exception('sdasd')
                    return act_patternStep,pathList
                # return pre_row,pre_col

        # while self._mat[pre_row][pre_col] == 0:
        pass

    def farAstar(self,c_row,c_col,t_row,t_col):
        pathList = []
        map2d = self._mat
        aStar = A.AStar(map2d, Point(c_row, c_col), Point(t_row, t_col))
        a_path = aStar.start()
        for unit in a_path:
            pathList.append((unit.x, unit.y))
        return pathList

    def drawPath(self):

        # self._pathLst
        pathLst = [[]  for i in range(self._robNum *2)]

        for robID in range(self._robNum):
            _path = self._pathLst[robID]
            pathLst[robID *2] = [unit[0]+0.5 for unit in _path]
            pathLst[robID *2 + 1] = [unit[1]+0.5 for unit in _path]
        drawPath.drawPath(self._ins,pathLst)

        print(self._coveredGrid)
        pass
        pass


if __name__ == '__main__':
    pass

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
    ins =    instance.MCMPInstance(row,col,obstacleLst,robPosLst)
    mcmp_decode = MCMPDecode(ins)

    chrom = []


    # print(np.random.randint(0,8,size = 5))
    # print(np.random.randint(0,20,size = 3))
    # print(np.random.randint(0,20,size = 3))
    # print(np.random.randint(0,120,size=5))

    codeNum = 10
    typeLst = np.random.randint(0, 8, size=robNum + codeNum)
    stepLst = np.random.randint(0,120,size=robNum  + codeNum)
    xLst = np.random.randint(0,20,size = codeNum)
    yLst = np.random.randint(0,20,size = codeNum)

    chrom = [typeLst[0],stepLst[0],typeLst[1],stepLst[1]]

    for i in range(codeNum):
        chrom.append(xLst[i])
        chrom.append(yLst[i])
        chrom.append(typeLst[i + robNum])
        chrom.append(stepLst[i + robNum])

    print(chrom)
    # chrom.append(type)
    # for chrom.append()
    mcmp_decode.decode(chrom)