from MCMPinstance import MCMPInstance
from stcMap.stcMap import  STC_Map,GridInd,STCGridInd,STCVertType,STCVert,STCVirtualVertType,\
    DirType,getDir
import  random
from enum import  Enum
from  drawEnv import drawPic,drawSTCPic,drawSTCGraph,drawEvalSTCGraph
from math import floor
from itertools import chain
import  networkx as nx
import  numpy as np
import  math
import  time
from MCMPastar import MCMP_Solver,STC_ASTAR
from stcMap.stc2path import STC2Path


class LawnMainDir(Enum):
    right = 1
    top  = 2
    left = 3
    bottom = 4

class STCEvaluator(object):
    def __init__(self,ins):
        self._ins = ins
        self._row = ins._row
        self._col = ins._col
        self._ins = ins
        self._mat = ins._mat
        self._robNum = ins._robNum
        self._robPosLst = ins._robPosLst
        self._robRowLst = ins._robRowLst
        self._robColLst = ins._robColLst
        self._robReachRowLst = ins._robReachRowLst
        self._robReachColLst = ins._robReachColLst

        self._s_map = STC_Map(self._ins)
        self._stcGraph = self._s_map._stcGraph
        self._vitualIndSet = self._s_map._vitualIndSet
        self._stcVitualIndSet = self._s_map._stcVitualIndSet
        self._waySTCNodeNum = self._s_map._waySTCNodeNum


        self._patternLst = []
        self._patternLst.append([LawnMainDir.right, LawnMainDir.top, LawnMainDir.bottom, LawnMainDir.left])
        self._patternLst.append([LawnMainDir.right, LawnMainDir.bottom, LawnMainDir.top, LawnMainDir.left])

        self._patternLst.append([LawnMainDir.left, LawnMainDir.top, LawnMainDir.bottom, LawnMainDir.right])
        self._patternLst.append([LawnMainDir.left, LawnMainDir.bottom, LawnMainDir.top, LawnMainDir.right])

        self._patternLst.append([LawnMainDir.top, LawnMainDir.left, LawnMainDir.right, LawnMainDir.bottom])
        self._patternLst.append([LawnMainDir.top, LawnMainDir.right, LawnMainDir.left, LawnMainDir.bottom])

        self._patternLst.append([LawnMainDir.bottom, LawnMainDir.left, LawnMainDir.right, LawnMainDir.top])
        self._patternLst.append([LawnMainDir.bottom, LawnMainDir.right, LawnMainDir.left, LawnMainDir.top])

        self._dirDic = dict()
        self._dirDic[LawnMainDir.right] = [LawnMainDir.top, LawnMainDir.right, LawnMainDir.bottom]
        self._dirDic[LawnMainDir.left] = [LawnMainDir.top, LawnMainDir.left, LawnMainDir.bottom]
        self._dirDic[LawnMainDir.top] = [LawnMainDir.left, LawnMainDir.top, LawnMainDir.right]
        self._dirDic[LawnMainDir.bottom] = [LawnMainDir.left, LawnMainDir.bottom, LawnMainDir.right]

        self._oppoDirDic = dict()
        self._oppoDirDic[LawnMainDir.right] = LawnMainDir.left
        self._oppoDirDic[LawnMainDir.left] = LawnMainDir.right
        self._oppoDirDic[LawnMainDir.top] = LawnMainDir.bottom
        self._oppoDirDic[LawnMainDir.bottom] = LawnMainDir.top

        self.stcConvertPath = STC2Path(self._ins, self._s_map)
        pass


    def evaluate(self, x):
        # print('waySTCNodeNum', self._waySTCNodeNum)
        # print(x)
        self.generateSTree(x)
        if len(self._coverSet) == self._waySTCNodeNum:
            makespan = self.stcConvertPath.generatePath(self._robStartSTCIndLst,
                                                   self._robStreeLst,
                                                   self._robSetLst)
            return True,makespan
        else:
            return False,np.inf
        # return makespan

    def generateSTree(self,x):
        self._robPatternLst = [[] for x in range(self._robNum)]
        for robID in range(self._robNum):
            for ind in x[robID]:
                firstOrdPos = ind[0]
                patternInd = floor(ind[1] / 0.125)
                rob_step = floor(ind[2] * self._stcGraph.number_of_nodes() * 0.5)
                self._robPatternLst[robID].append((firstOrdPos, patternInd, rob_step))

        # robID = 0
        # for ind in x:
        #     # main_dir,first_dir = self._patternLst[floor(ind[0]/0.125)]
        #     firstOrdPos = ind[0]
        #     patternInd = floor(ind[1]/0.125)
        #     '''
        #     感觉没有必要那么大
        #     '''
        #     rob_step = floor(ind[2] * self._stcGraph.number_of_nodes() * 0.5)
        #     self._robPatternLst[robID].append((firstOrdPos,patternInd,rob_step))
        #
        #     if robID == (self._robNum-1):
        #         robID = 0
        #     else:
        #         robID += 1
        # print(self._robPatternLst)
        # print(len(self._robPatternLst[0]))
        # print(len(self._robPatternLst[1]))
        self._robSetLst = [[] for x in range(self._robNum)]
        self._robStartSTCIndLst = []
        self._coverSet = set()

        for robID in range(self._robNum):
            stc_ind  = self._s_map.gridInd2STCGridInd(GridInd(row = self._robPosLst[robID][0],
                                                          col = self._robPosLst[robID][1]))
            self._robSetLst[robID].append(stc_ind)
            self._robStartSTCIndLst.append(stc_ind)
            self._coverSet.add(stc_ind)

        self._c_robPosLst = []
        self._c_robDirLst = []
        self._p_robDirLst = []
        self._c_mainDirLst = []
        self._c_robStepLst = [0 for x in range(self._robNum)]
        self._robStepLst  = []
        self._robPatternStepLst = [0 for x in range(self._robNum)]
        self._robStreeLst = [nx.Graph() for x in range(self._robNum)]
        self._c_sortPatternIndLst = []
        self._robEndLst = [False for x in range(self._robNum)]

        # for robID,x in enumerate(self._robPatternLst):
        #     # print(x[0])
        #     deg_file.write('robID'+str(robID)+'\n' )
        #     for p in x:
        #         deg_file.write(str(p)+'\n')
        # deg_file.flush()

        for robID in range(self._robNum):
            firstOrdPos,patternInd,rob_step = self._robPatternLst[robID][0]
            firstBool,p_pos,pos = self.getNextBranchPos(robID,firstOrdPos)
            # c_pos,pos = self.getFirstPos(robID,main_dir,first_dir)
            # print('first_pos', pos)
            self._c_sortPatternIndLst.append(patternInd)
            self._c_robPosLst.append(pos)
            # self._c_robDirLst.append(first_dir)
            # self._p_robDirLst.append(first_dir)
            # self._c_mainDirLst.append(main_dir)
            self._robStepLst.append(rob_step)
            self._robSetLst[robID].append(pos)
            # self._robStreeLst[robID].add_node(pos)
            self._robStreeLst[robID].add_edge(p_pos,pos)
            self._coverSet.add(pos)



        # raise  Exception('XX')
        # print(self._robPatternLst)
        pass
            # for y in x:
            #     print(y)
        circleTime = 0
        while False in self._robEndLst:
            for robID in range(self._robNum):
                if self._robEndLst[robID]:
                    continue
                pos = self.getUncoverPos(robID)
                if (pos.row == -1 and  pos.col == -1) or self._c_robStepLst[robID] > self._robStepLst[robID]:
                    self._robPatternStepLst[robID] += 1
                    # print('robID ', robID, ' PatternStep ', self._robPatternStepLst[robID])
                    robPatternStep = self._robPatternStepLst[robID]
                    # print('robID = ',robID, ' robPatternStep = ', robPatternStep)
                    if robPatternStep < len(self._robPatternLst[robID]):
                        firstOrdPos, patternInd, rob_step = self._robPatternLst[robID][robPatternStep]
                        firstBool, p_pos,f_pos = self.getNextBranchPos(robID,firstOrdPos)
                    else:
                        firstBool = False
                    if firstBool:
                        self._coverSet.add(f_pos)
                        self._robSetLst[robID].append(f_pos)
                        self._c_robPosLst[robID] = f_pos
                        self._c_sortPatternIndLst[robID] = patternInd
                        self._c_robStepLst[robID] = 0
                        self._robStreeLst[robID].add_edge(p_pos, f_pos)
                    else:
                        self._robEndLst[robID] = True
                        # raise  Exception('xx')
                else:
                    c_pos = self._c_robPosLst[robID]
                    self._robStreeLst[robID].add_edge(c_pos,pos)
                    self._c_robStepLst[robID] += 1
                    self._robSetLst[robID].append(pos)
                    self._c_robPosLst[robID] = pos
                    self._coverSet.add(pos)
                    # print(pos)
            circleTime += 1
            # print('circletime = ',circleTime)
            # if circleTime == 47:
            #     print('47')
            #     pass
            if circleTime > 200:
                break
        # print('._coverSet = ',len(self._coverSet))

    def getUncoverPos(self,robID):
        c_pos = self._c_robPosLst[robID]
        # c_robDir = self._c_robDirLst[robID]
        # p_robDir = self._p_robDirLst[robID]
        # c_mainDir = self._c_mainDirLst[robID]
        c_sPatternInd = self._c_sortPatternIndLst[robID]
        sortDirLst = self._patternLst[c_sPatternInd]

        for dir in sortDirLst:
            pos = self.getNextUnCoverLawnPos(c_pos,dir)
            if pos.row == -1 and pos.col == -1:
                continue
            return pos
        return pos

    def getNextBranchPos(self,robID, var):
        robSet = self._robSetLst[robID]
        robNeiLst = []
        for stc_ind in robSet:
            # print('stc_ind = ', stc_ind)
            neiLst = self._stcGraph.neighbors(stc_ind)
            for neiInd in neiLst:
                if neiInd not in self._coverSet:
                    # print('neiInd = ',neiInd)
                    dis = self.calBranchDis(robID,neiInd)
                    robNeiLst.append((stc_ind, neiInd, dis))
                    # print(abs(angle -base_angle))
        robNeiNum  = len(robNeiLst)
        if robNeiNum != 0 :
            robNeiSet = sorted(robNeiLst, key = lambda  x: x[2])
            resInd = floor(robNeiNum * var)
            return True,robNeiSet[resInd][0],robNeiSet[resInd][1]
        else:
            return False, None, None


    def calAngle(self,robID, stc_ind):
        rob_stc_ind = self._robStartSTCIndLst[robID]
        base_x = self._stcGraph.nodes[rob_stc_ind]['vert']._pos_x
        base_y = self._stcGraph.nodes[rob_stc_ind]['vert']._pos_y

        ind_x = self._stcGraph.nodes[stc_ind]['vert']._pos_x
        ind_y = self._stcGraph.nodes[stc_ind]['vert']._pos_y
        y = ind_y - base_x
        x = ind_x - base_x

        if ind_y > base_y:
            return math.atan2(y,x)
        else:
            return math.atan2(y,x) + math.pi

    def calBranchDis(self,robID,stc_ind):
        # dis = 0
        vec_base = np.array([self._stcGraph.nodes[stc_ind]['vert']._pos_x, self._stcGraph.nodes[stc_ind]['vert']._pos_y])
        rob_stc_ind = self._robStartSTCIndLst[robID]
        vec = np.array([self._stcGraph.nodes[rob_stc_ind]['vert']._pos_x, self._stcGraph.nodes[rob_stc_ind]['vert']._pos_y])
        dis = np.linalg.norm(vec_base - vec)
        return dis


    def getFirstPosByDis(self,robID, var):
        robSet = self._robSetLst[robID]
        robNeiSet = []
        for stc_ind in robSet:
            neiLst = self._stcGraph.neighbors(stc_ind)
            for neiInd in neiLst:
                if neiInd not in self._coverSet:
                    dis = self.calFirstDis(robID,neiInd)
                    robNeiSet.append((neiInd,dis))
        robNeiNum  = len(robNeiSet)
        if robNeiNum != 0 :
        # print(robNeiSet)
            robNeiSet = sorted(robNeiSet, key = lambda  x: x[1])
            resInd = floor(robNeiNum * var)
            return True,robNeiSet[resInd][0]
        else:
            return False, None
    def calFirstDis(self,robID,stc_ind):
        dis = 0
        vec_base = np.array([self._stcGraph.nodes[stc_ind]['vert']._pos_x, self._stcGraph.nodes[stc_ind]['vert']._pos_y])


        for x in range(self._robNum):
            if x == robID:
                continue
            rob_stc_ind = self._robStartSTCIndLst[x]
            vec = np.array([self._stcGraph.nodes[rob_stc_ind]['vert']._pos_x, self._stcGraph.nodes[rob_stc_ind]['vert']._pos_y])
            dis += np.linalg.norm(vec_base - vec)
        return dis

    def getFirstPos(self,robID,mainDir, firstDir):
        c_pos = self._robStartSTCIndLst[robID]
        c_robDir = firstDir
        p_robDir = firstDir

        while True:
            pos = self.getNextLawnPos(c_pos,c_robDir)
            # print(pos)
            if pos.row == -1 and pos.col == -1:
                p_robDir = c_robDir
                c_robDir = self.getNextDirPos(mainDir,c_robDir,p_robDir)
                if c_robDir == mainDir:
                    pos = self.getNextLawnPos(c_pos,c_robDir)
                    if pos.row == -1 and pos.col == -1:
                        return c_pos, pos
                        # raise Exception('xx')
                        # break
                    else:
                        if pos not in self._coverSet:
                            return c_pos,pos
                        c_pos = pos

                        c_robDir = self.getNextDirPos(mainDir, c_robDir, p_robDir)
            else:
                if pos not in self._coverSet:
                    return c_pos, pos
                c_pos = pos


    def getNextLawnPos(self,stc_ind :STCGridInd, dir :LawnMainDir):
        res = STCGridInd(-1,-1,STCVirtualVertType.NoVir)
        neiLst = self._stcGraph.neighbors(stc_ind)
        if dir == LawnMainDir.right:
            for nei in neiLst:
                if stc_ind.row + 1 == nei.row and stc_ind.col == nei.col:
                    return nei
            return res
        if dir == LawnMainDir.left:
            for nei in neiLst:
                if stc_ind.row + 1 == nei.row and stc_ind.col == nei.col:
                    return nei
            return res

        if dir == LawnMainDir.top:
            for nei in neiLst:
                if stc_ind.col + 1 == nei.col and stc_ind.row == nei.row:
                    return nei
            return res

        if dir == LawnMainDir.bottom:
            for nei in neiLst:
                if stc_ind.col - 1 == nei.col and stc_ind.row == nei.row:
                    return nei
            return res

    def getNextUnCoverLawnPos(self,stc_ind :STCGridInd, dir :LawnMainDir):
        res = STCGridInd(-1,-1,STCVirtualVertType.NoVir)
        neiLst = self._stcGraph.neighbors(stc_ind)
        if dir == LawnMainDir.right:
            for nei in neiLst:
                if nei not in self._coverSet:
                    if stc_ind.row + 1 == nei.row and stc_ind.col == nei.col:
                        return nei
            return res
        if dir == LawnMainDir.left:
            for nei in neiLst:
                if nei not in self._coverSet:
                    if stc_ind.row + 1 == nei.row and stc_ind.col == nei.col:
                        return nei
            return res

        if dir == LawnMainDir.top:
            for nei in neiLst:
                if nei not in self._coverSet:
                    if stc_ind.col + 1 == nei.col and stc_ind.row == nei.row:
                        return nei
            return res

        if dir == LawnMainDir.bottom:
            for nei in neiLst:
                if nei not in self._coverSet:
                    if stc_ind.col - 1 == nei.col and stc_ind.row == nei.row:
                        return nei
            return res

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

    def inRobSet(self,stc_ind:STCGridInd):

        if stc_ind in self._coverSet:
            return True
        return False

    def allCover(self):
        if len(self._coverSet) == self._waySTCNodeNum:
            return True
        else:
            return False
        # self._coverSet = set()


# def stcEvaluator(pop):
#     fitness  = 0
#     for ind in pop:
#         for x in ind:
#             fitness += x
#     return fitness

if __name__ == '__main__':
    ins = MCMPInstance()

    ins.loadCfg('D:\\py_code\\MCMP_encode\\benchmark\\r4_r50_c50_p0.8_s1000_Outdoor_Cfg.dat')
    # ins.loadCfg('D:\\py_code\\MCMP_encode\\benchmark\\r2_r40_c20_p0.9_s1000_Outdoor_Cfg.dat')
    # ins.loadCfg('D:\\py_code\\MCMP_encode\\benchmark\\r2_r20_c20_p0.9_s1000_Outdoor_Cfg.dat')
    stc_eval = STCEvaluator(ins)
    # random.seed(6)
    # pop = []
    # for i in range(100):
    #     pop.append((random.random(),random.random(),random.random()))
    for seed in range(100):
        random.seed(seed)
        random.seed(33)
        print(seed)
        pop = []
        for i in range(300):
            pop.append((random.random(),random.random(),random.random()))

        # pop = [[0.10754700880355095, 0.3747030205016403, 0.8467029952096821], [0.8179480602067888, 0.28978161459048557, -0.061817810733454104], [0.2550690257394217, 0.49543508709194095, 0.4494910647887381], [0.651592972722763, 0.7887233511355132, 0.0938595867742349], [0.02834747652200631, 0.8357651039198697, 0.43276706790505337], [0.762280082457942, 0.0021060533511106927, 0.4453871940548014], [0.7215400323407826, 0.22876222127045265, 0.9452706955539223], [0.5075105415233063, 0.47988597075501366, 0.5209384176131452], [0.5414124727934966, 0.9391491627785106, 0.38120423768821243], [0.21659939713061338, 0.4221165755827173, 0.029040787574867943], [0.22319578024667075, 0.7116036647979832, 0.3948980098582996], [0.22169166627303505, 0.43788759365057206, 0.49581224138185065], [0.23308445025757263, 0.2308665415409843, 0.2187810373376886], [0.4596034657377336, 0.28978161459048557, 0.021489705265908876], [0.38655710476146987, 0.4209186792090759, 0.42082999993781356], [0.1859062658947177, 0.9925434121760651, 0.8599465287952899], [0.12088995980580641, 0.3326951853601291, 0.7214844075832684], [0.7215400323407826, 0.22876222127045265, 0.6370298197074602], [0.7111917696952796, 0.9364405867994596, 0.4221069999614152], [0.830035693274327, 0.670305566414071, 0.3033685109329176], [0.5875806061435594, 0.8824790008318577, 0.8461974184283128], [0.9540841761007466, 0.6485064180992564, 0.3948980098582996], [0.24273997354306764, 0.7974042475543028, 0.4143139993007743], [0.17300740157905092, 0.548798761388153, 0.7030407620656315], [0.1941186449851896, 0.573771199478443, 0.6659575282786826], [0.33204516274256446, 0.03791653059858058, 0.8194141106127972], [0.5084264882499818, 0.7784426150001458, 0.5209384176131452], [0.39325509496422606, 0.4896935204622582, 0.029574963966907064], [0.04348729035652743, 0.703382088603836, 0.9831877173096739], [0.4581468000997244, 0.027974984083842358, 0.40836419548741143], [0.5931837303800576, 0.393599686377914, 0.17034919685568128], [0.5022385584334831, 0.9820766375385342, 0.7705231398308006], [0.5396174484497788, 0.8602897789205496, 0.23217612806301458], [0.882696930394086, 0.47701009597226784, 0.10012914395045203], [0.1859062658947177, 0.9925434121760651, 0.8982921034208464], [0.513771663187637, 0.9524673882682695, 0.5777948078012031], [0.45913173191066836, 0.2692794774414212, 0.5479963094662489], [0.9571162814602269, 0.005709129450392925, 0.7836552326153898], [0.8204859119254819, 0.8861795808260082, 0.7405034118331963], [0.4260906796881502, 0.05612329752074041, 0.8700101551766398], [0.48492511222773416, 0.3567899645449557, 0.3460779190181549], [0.5384787957378443, 0.6234894527975051, 0.6124524647827256], [0.4581468000997244, 0.027974984083842358, 0.22960503127702392], [0.1772112589385827, 0.5844608707784413, 0.8610088608533248], [0.798438940577426, 0.7970975626354962, 0.8164373705606909], [0.25529404008730594, 0.841744832274096, 0.6731135254387071], [0.08323413780389788, 0.0166906301155596, 0.014559974924812313], [0.42881224677206103, 0.4210135874302673, 0.1155581805922733], [0.6248020841524763, 0.3444228640964949, 0.06951537853084733], [0.1596255246938475, 0.5273803990480128, 0.16814494622242826], [0.2729144368186801, 0.7115899271852729, 0.4547016300456639], [0.3220017663873259, 0.4737710141702789, 0.023634577631987064], [0.38655710476146987, 0.4209186792090759, 0.18803930475131292], [0.10876169244541334, 0.8998185003560202, 0.5101159809286764], [0.2090909925517701, 0.6056486400340165, 0.8170396683778869], [0.020818108509287336, 0.017864520827795327, 0.146461740399346], [0.7188354727617898, 0.16022759262970465, 0.7046056278520025], [0.6781757952769475, 0.5447021635789044, 0.22059974802267657], [0.9755945178178834, 0.797810857706151, 0.516599516949393], [0.22319578024667075, 0.6485064180992564, 0.3948980098582996], [0.5758459627880567, 0.32124580934512525, 0.6309478612713469], [0.058785116206491295, 0.29860594962301334, 0.9679033101508892], [0.8755342442351592, 0.30638662033324593, 0.8585144063565593], [0.31036362735313405, 0.9392884321352825, 0.7438421186671211], [0.4161722627650255, 0.25235810227983535, 0.008480262463668842], [0.8787178982088466, 0.03791653059858058, 0.8194141106127972], [0.962201125180818, 0.5702805702451802, 0.17151709517771863], [0.8677810644349934, 0.9737752361596916, 0.7040231423300713], [0.5088737460778905, 0.37796883434360806, 0.34693088456262167], [0.2057617572947047, 0.6741530142468641, 0.4329501211003163], [0.1941186449851896, 0.10442422284151531, 0.6659575282786826], [0.29607267308315155, 0.4997999222368016, 0.3253456548759963], [0.8716215074235552, 0.8996782696347811, 0.018092983640471738], [0.2008530114407594, 0.3277407050962675, 0.9870497179280261], [0.7827003757293756, 0.3390956478509337, 0.21302979638081376], [0.6744550697237632, 0.8377010701539643, 0.9321874718936273], [0.3438498147908198, 0.8823932024664636, 0.6871101821536574], [0.48449872261249405, 0.9855082298257978, 0.23464043487103847], [0.7254651862412724, 0.0846802304164842, 0.16969414179438758], [0.9109877835080679, 0.21296819499142416, 0.7591161827164402], [0.6002088301322496, 0.8411321957058551, 0.3681079994056491], [0.34028523500198804, 0.29121528741113467, 0.8674198235869027], [0.6039825288917112, 0.9543074571721899, 0.8872651047169627], [0.13534597739545295, 0.5511704740692165, 0.1042749980146136], [0.03913779859691058, 0.07319341883234853, 0.866168357366572], [0.7881164487252263, 0.8285059714691135, 0.3408974641165834], [0.6151860325590366, 0.7819036016327547, 0.3780396288383874], [0.5707815255990233, 0.2237140727487692, 0.08174326235239371], [0.26672364298173634, 0.8907681278553053, 0.5644468332401974], [0.9250672021084733, 0.4577692590412453, 0.2771827661076983], [0.7870146635603288, 0.8277681566457297, 0.012381744486666624], [0.670411639023931, 0.09168312261651779, 0.1151024984279273], [0.8850600703796611, 0.04002353689016469, 0.2396333648675093], [0.9881584986060327, 0.4210135874302673, 0.1155581805922733], [0.16738343746133177, 0.24142028509784308, 0.7440064165370084], [0.1028341459863098, 0.9107644182793333, 0.3782772705442261], [0.9702640365282106, 0.9092227281507113, 0.29402358494854774], [0.2534101360411267, 0.47701009597226784, 0.10012914395045203], [0.6520501994894172, 0.039620213413704475, 0.010506151518672291], [0.9825836265504634, 0.2955498600489178, 0.5965706431884413], [0.44984453463009777, 0.31328086106892794, 0.06296479004764532], [0.9133920171659404, 0.9698132768381156, 0.9697965044964699], [0.1113623101268919, 0.21519327003609845, 0.6178068800115557], [0.979952885890077, 0.5429131974847156, 0.6881898080477126], [0.6618344288753493, 0.259085991853645, 0.5416022629129655]]

    # print(pop)
    #     e_start = time .clock()
    #     makespan = stc_eval.evaluate(pop)
    #     print('makespan = ', makespan)
    #     e_end = time .clock()
    #     print('evaluate time = ', e_end - e_start)
        e_start = time .clock()
        try:
            stc_eval.evaluate(pop)
            pathLst = stc_eval.stcConvertPath._pathLst
        except Exception as e:
            pathLst = stc_eval.stcConvertPath._virPathLst
            print(pathLst)
            print(e)
        e_end = time.clock()
        # stc_eval.evaluate(pop)
        print('evaluate time = ', e_end - e_start)

        #     pass

    # allEdgeLstPnt = []
    # _graph = stc_eval._stcGraph
    # for edge in stc_eval._stcGraph.edges():
    #     # print(edge)
    #     sPnt_x = _graph.nodes[edge[0]]['vert']._pos_x
    #     sPnt_y = _graph.nodes[edge[0]]['vert']._pos_y
    #     tPnt_x = _graph.nodes[edge[1]]['vert']._pos_x
    #     tPnt_y = _graph.nodes[edge[1]]['vert']._pos_y
    #     # edgeLst.append((t_pos_x, t_pos_y, s_pos_x, s_pos_y))
    #
    #     allEdgeLstPnt.append((sPnt_x, sPnt_y, tPnt_x, tPnt_y))
    # drawSTCPic(ins,allEdgeLstPnt)
        stcGraphLst = []
        allEdgeLstPnt = []
        for robID in range(stc_eval._robNum):
            # print(stc_eval._robSetLst)
            _robSet  = stc_eval._robSetLst[robID]
            _stcGraph = []
            for stcGridInd in _robSet:
                _stcGraph.append((stcGridInd.row *2 , stcGridInd.col *2))
            stcGraphLst.append(_stcGraph)

            stree = stc_eval._robStreeLst[robID]
            edgeLst = []
            for edge in stree.edges():
                # print(edge)
                t_pos_x = stc_eval._stcGraph.nodes[edge[0]]['vert']._pos_x
                t_pos_y = stc_eval._stcGraph.nodes[edge[0]]['vert']._pos_y

                s_pos_x = stc_eval._stcGraph.nodes[edge[1]]['vert']._pos_x
                s_pos_y = stc_eval._stcGraph.nodes[edge[1]]['vert']._pos_y

                edgeLst.append((t_pos_x,t_pos_y,s_pos_x,s_pos_y))
            allEdgeLstPnt.append(edgeLst)
        # print(stcGraphLst)
        # drawEvalSTCGraph(ins,stcGraphLst= stcGraphLst, edgePntLst = allEdgeLstPnt)
        drawEvalSTCGraph(ins,stcGraphLst= stcGraphLst, edgePntLst = allEdgeLstPnt, multiPath= pathLst)
    exit()
    drawEvalSTCGraph(ins,stcGraphLst= stcGraphLst, edgePntLst = allEdgeLstPnt, multiPath= stc_eval.stcConvertPath._pathLst)
        # drawPic(ins)
