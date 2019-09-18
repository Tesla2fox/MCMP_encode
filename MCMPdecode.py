import numpy
import drawEnv
from MCMPastar import MCMP_Solver
from MCMPinstance import MCMPInstance
from drawEnv import  drawPic
from enum import  Enum
from collections import  namedtuple
import numpy as np


GridInd = namedtuple('GridInd', ['row', 'col'])
RobPos = namedtuple('RobPos',['row','col'])


class LawnMainDir(Enum):
    right = 1
    top  = 2
    left = 3
    bottom = 4

class RobMode(Enum):
    LAWN = 1
    ASTAR = 2
    STOP = 3
    FARASTAR = 4

class MCMP_Decode(object):
    def __init__(self,ins: MCMPInstance):
        self._ins = ins
        self._row = ins._row
        self._col = ins._col
        self._ins = ins
        self._mat = ins._mat
        self._robNum = ins._robNum
        self._robPosLst = ins._robPosLst

        self._robPosLst = []
        for pos in ins._robPosLst:
            self._robPosLst.append(RobPos(row= pos[0], col = pos[1]))

        print(self._robPosLst)
        self._robRowLst = ins._robRowLst
        self._robColLst = ins._robColLst
        self._robReachRowLst = ins._robReachRowLst
        self._robReachColLst = ins._robReachColLst


        self._mcmp_astar = MCMP_Solver(ins)


        self._parternLst = []
        self._parternLst.append([LawnMainDir.right,LawnMainDir.top])
        self._parternLst.append([LawnMainDir.right,LawnMainDir.bottom])

        self._parternLst.append([LawnMainDir.left,LawnMainDir.top])
        self._parternLst.append([LawnMainDir.left,LawnMainDir.bottom])

        self._parternLst.append([LawnMainDir.top,LawnMainDir.left])
        self._parternLst.append([LawnMainDir.top,LawnMainDir.right])


        self._parternLst.append([LawnMainDir.bottom,LawnMainDir.left])
        self._parternLst.append([LawnMainDir.bottom,LawnMainDir.right])

        print(self._mcmp_astar)


        self._dirDic = dict()
        self._dirDic[LawnMainDir.right] = [LawnMainDir.top, LawnMainDir.right, LawnMainDir.bottom]
        self._dirDic[LawnMainDir.left] = [LawnMainDir.top, LawnMainDir.left, LawnMainDir.bottom]
        self._dirDic[LawnMainDir.top] = [LawnMainDir.left, LawnMainDir.top, LawnMainDir.right]
        self._dirDic[LawnMainDir.bottom] = [LawnMainDir.left, LawnMainDir.bottom, LawnMainDir.right]



    def getNextLawnPos(self,robPos: RobPos,dir):
        # resPos = [-1,-1]
        # print(robPos)
        resPos = RobPos(-1, -1)
        if dir == LawnMainDir.right:
            if robPos.row < self._row - 1:
               resPos = RobPos(robPos.row + 1, robPos.col)
        if dir == LawnMainDir.left:
            if robPos.row > 0:
               resPos = RobPos(robPos.row - 1, robPos.col)
        if dir == LawnMainDir.top:
            if robPos.col < self._col - 1:
               resPos = RobPos(robPos.row, robPos.col + 1)
        if dir == LawnMainDir.bottom:
            if robPos.col > 0 :
               resPos = RobPos(robPos.row, robPos.col - 1)
        return resPos

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
            if pos == RobPos(-1, -1):
                p_robDir = c_robDir
                c_robDir = self.getNextDirPos(mainDir,c_robDir,p_robDir)
                # print(c_robDir)
                # raise Exception('xx')
                if c_robDir == mainDir:
                    pos = self.getNextLawnPos(c_Pos, c_robDir)
                    if pos == RobPos(-1, -1):
                        break
                    else:
                        resPathLst.append(pos)
                        c_Pos = pos
                        c_robDir = self.getNextDirPos(mainDir, c_robDir,p_robDir)
            else:
                resPathLst.append(pos)
                c_Pos = pos
        # print('robID = ',robID, resPathLst)
        return resPathLst

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

    def decode(self,chrom):

        # print('begin')
        lvlBool = True


        self._predictPathLst = []
        self._robState = []
        self._c_robPosLst = []
        self._pathLst = [[] for i in range(self._robNum)]
        self._midPosLst = [[] for i in range(self._robNum)]
        self._NextState = [[] for x in range(self._robNum)]
        self._c_robPosLst = [robPos for robPos in self._robPosLst]


        self._coverSet = set()
        self._unCoverSet = set()
        for i in range(len(self._robReachRowLst)):
            self._unCoverSet.add(RobPos(row= self._robReachRowLst[i], col = self._robReachColLst[i]))

        chromCircleTime = 0
        encodeEnd = False
        while True:
            for robID in range(self._robNum):
                if chromCircleTime * self._robNum* 5  + robID *4 + self._robNum*2 >= len(chrom):
                    encodeEnd = True
                    break
                else:
                    dic = dict()
                    dic['row'] = chrom[chromCircleTime * self._robNum * 5  + self._robNum *2  + robID * 5]
                    dic['col'] = chrom[chromCircleTime * self._robNum * 5  + self._robNum *2  + robID * 5 + 1]
                    dic['patternType'] = chrom[chromCircleTime * self._robNum * 5  + self._robNum *2  + robID * 5 + 2]
                    dic['step'] = chrom[chromCircleTime * self._robNum * 5  + self._robNum *2  + robID * 5 + 3]
                    dic['lvl'] = chrom[chromCircleTime * self._robNum * 5  + self._robNum *2  + robID * 5 + 4]
                    self._NextState[robID].append(dic)
            if encodeEnd:
                break
            chromCircleTime  += 1
        '''
        此处没必要
        '''
        # for robID in range(self._robNum):
        #     self._NextState[robID][-1]['lvl'] = len(self._robReachRowLst)
        # print('len = ', len(self._NextState[0]))
        # print(self._NextState[0])
        # print(self._NextState[1])
        # raise  Exception('xx')
        for robID in range(self._robNum):
            dic = dict()
            dic['patternType'] = chrom[2*robID]
            dic['step'] = chrom[2*robID + 1]
            dic['actPatternStep'] = 0
            dic['mode']  = RobMode.LAWN
            dic['farStep'] = 0
            self._robState.append(dic)
            main_dir, first_dir = self._parternLst[dic['patternType']]
            self.addPath(robID= robID, robPos= self._robPosLst[robID])
            predictPath = self.getWholeLawnPos(robID,main_dir,first_dir,dic['step'])
            # print('robID',robID,' mainDir = ', main_dir,' first_dir = ',first_dir)
            # drawPic(ins,titleName= str(main_dir) +' ' + str(first_dir),singlePathInd= predictPath)
            self._predictPathLst.append(predictPath)

        # print('初始化完成')

        circleTime = 0
        while (not self.allRobStop() and not self.allcover()):
            # print('circleTime = ', circleTime)
            circleTime += 1
            for robID in range(self._robNum):
                state = self._robState[robID]
                c_mode = state['mode']
                if c_mode == RobMode.LAWN:
                    act_patternStep = state['actPatternStep']
                    act_patternStep,pathLst = self.getPredictPath(robID,act_patternStep)
                    state['actPatternStep'] = act_patternStep
                    '''
                    no obstacle
                    '''
                    if len(pathLst) == 1:
                        self.addPath(robID=robID, robPos= pathLst[0])
                    elif len(pathLst) == 0:
                        farStep = state['farStep']
                        # print('robID ', robID)
                        # print('farStep ',farStep)

                        if state['farStep'] == len(self._NextState[robID]):
                            state['mode'] = RobMode.STOP
                            continue
                        if lvlBool:
                            lvl = self._NextState[robID][farStep]['lvl']
                            lst = []
                            for pos in self._unCoverSet:
                                dis = abs(self._c_robPosLst[robID].row - pos.row) + abs(self._c_robPosLst[robID].col - pos.col)
                                lst.append((pos,dis))
                            lst = sorted(lst,key = lambda  x : x[1])
                            if len(lst) == 0:
                                break
                            if lvl >= len(lst):
                                t_pos = lst[0][0]
                            else:
                                t_pos = lst[lvl][0]
                            t_row, t_col = t_pos
                            pass
                        else:
                            while True:
                                if state['farStep'] == len(self._NextState[robID]):
                                    state['mode'] = RobMode.STOP
                                    break
                                t_row = self._NextState[robID][farStep]['row']
                                t_col = self._NextState[robID][farStep]['col']
                                if self._mat[t_row][t_col] == 1:
                                    state['farStep'] += 1
                                    continue
                                else:
                                    break
                        # if state['mode'] == RobMode.STOP:
                        #     break
                        farPathLst = list(self._mcmp_astar.astar(start = (self._c_robPosLst[robID]),goal= (t_row, t_col)))
                        farPathLst = [RobPos(unit[0], unit[1]) for unit in farPathLst]
                        self._midPosLst[robID].append(self._c_robPosLst[robID])
                        self._midPosLst[robID].append(RobPos(t_row, t_col))
                        # if True:
                        #     pass
                        # if farPathLst
                        state['mode'] = RobMode.FARASTAR
                        # print(list(farPathLst))
                        # print(self._pathLst[robID])
                        self.addPath(robID=robID, robPos= farPathLst[0])

                        state['far_astarStep'] = 1
                        state['far_astarPath'] = farPathLst
                        state['actPatternStep'] = 0

                        state['patternType'] = self._NextState[robID][farStep]['patternType']
                        state['step'] = self._NextState[robID][farStep]['step']

                        state['farStep'] += 1
                        # print(state['farStep'])
                        # break
                        # break
                    else:
                        state['mode'] = RobMode.ASTAR
                        self.addPath(robID=robID, robPos= pathLst[0])
                        state['astarStep'] = 1
                        state['astarPath'] = pathLst
                if c_mode  == RobMode.ASTAR:
                    astarStep = state['astarStep']
                    self.addPath(robID=robID, robPos= state['astarPath'][astarStep])
                    state['astarStep'] += 1
                    if state['astarStep'] == len(state['astarPath']):
                        state['mode'] = RobMode.LAWN
                if c_mode == RobMode.FARASTAR:
                    far_astarStep = state['far_astarStep']
                    self.addPath(robID=robID, robPos= state['far_astarPath'][far_astarStep])
                    state['far_astarStep'] += 1
                    if state['far_astarStep'] == len(state['far_astarPath']):
                        state['mode'] = RobMode.LAWN
                        main_dir, first_dir = self._parternLst[dic['patternType']]
                        # self._c_robPosLst.append((self._robRowLst[robID], self._robColLst[robID]))
                        predictPath = self.getWholeLawnPos(robID, main_dir, first_dir, dic['step'])
                        # print('robID', robID, ' mainDir = ', main_dir, ' first_dir = ', first_dir)
                        self._predictPathLst[robID] = predictPath

            if circleTime > 2000:
                # raise  Exception('xx')
                break

        makespan = len(max(self._pathLst, key = lambda  x : len(x)))
        c_ratio = len(self._coverSet)/len(self._robReachRowLst)

        # print('makespan = ', makespan)
        # print('c_ratio = ', c_ratio)


        # print(len(self._pathLst[0]))
        # print('')
        # drawPic(self._ins,multiPath=self._pathLst)
        # drawPic(self._ins,multiPath=self._pathLst, midPosLst= self._midPosLst)

        return makespan/c_ratio
        pass


    def allcover(self):
        if len(self._unCoverSet) == 0:
            return True
        else:
            return False
    def allRobStop(self):
        for state in self._robState:
            if state['mode'] != RobMode.STOP:
                return False
            else:
                return True

    def addPath(self,robID, robPos: RobPos):
        self._pathLst[robID].append(robPos)
        self._coverSet.add(robPos)
        self._c_robPosLst[robID] = robPos
        if robPos in self._unCoverSet:
            self._unCoverSet.remove(robPos)
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
            if self._mat[pre_row][pre_col] == 0 and RobPos(pre_row,pre_col) not in self._coverSet:
                row_bias = abs(pre_row - c_row)
                col_bias = abs(pre_col - c_col)
                if (row_bias + col_bias) == 1:
                    pathList.append(RobPos(pre_row,pre_col))
                    return act_patternStep,pathList
                    pass
                else:
                    map2d = self._mat
                    foundPath = self._mcmp_astar.astar(start = (c_row,c_col), goal= (pre_row,pre_col))
                    # aStar = A.AStar(map2d,Point(c_row,c_col),Point(pre_row,pre_col))
                    # a_path = aStar.start()
                    foundPath = [RobPos(unit[0],unit[1]) for unit in foundPath]
                    pathList.extend(foundPath)
                    # for unit in foundPath:
                    #     pathList.append((unit.,unit.y))
                    # print(pathList)
                    # raise  Exception('sdasd')
                    return act_patternStep,pathList

    def __str__(self):
        return str(self._ins)


if __name__ == '__main__':
    ins = MCMPInstance()
    ins.loadCfg('.\\benchmark\\r2_r40_c20_p0.9_s1000_Outdoor_Cfg.dat')
    decode = MCMP_Decode(ins)
    print(decode)
    # drawPic(ins,titleName='Instance')
    # chrom = [0, 1000, 5, 2000]
    # decode.decode(chrom)
    numpy.random.seed(5)
    codeNum = 30
    typeLst = np.random.randint(0, 8, size=ins._robNum+ codeNum)
    stepLst = np.random.randint(1,200,size=ins._robNum  + codeNum)
    xLst = np.random.randint(0,20,size = codeNum)
    yLst = np.random.randint(0,20,size = codeNum)
    lvl = np.random.randint(0,20,size = codeNum)

    randChrom = [typeLst[0],stepLst[0],typeLst[1],stepLst[1]]

    for i in range(codeNum):
        randChrom.append(xLst[i])
        randChrom.append(yLst[i])
        randChrom.append(typeLst[i + ins._robNum])
        randChrom.append(stepLst[i + ins._robNum])
        randChrom.append(lvl[i])

    print('randChrom = ', randChrom)

    decode.decode(randChrom)



    codeNum = 30
    typeLst = np.random.randint(0, 8, size=ins._robNum+ codeNum)
    stepLst = np.random.randint(1,200,size=ins._robNum  + codeNum)
    xLst = np.random.randint(0,20,size = codeNum)
    yLst = np.random.randint(0,20,size = codeNum)
    lvl = np.random.randint(0,20,size = codeNum)

    randChrom = [typeLst[0],stepLst[0],typeLst[1],stepLst[1]]

    for i in range(codeNum):
        randChrom.append(xLst[i])
        randChrom.append(yLst[i])
        randChrom.append(typeLst[i + ins._robNum])
        randChrom.append(stepLst[i + ins._robNum])
        randChrom.append(lvl[i])

    print('randChrom = ', randChrom)

    decode.decode(randChrom)



    # print(chrom)