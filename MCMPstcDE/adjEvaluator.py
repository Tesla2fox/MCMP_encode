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
import functools


# class LawnMainDir(Enum):
#     right = 1
#     top  = 2
#     left = 3
#     bottom = 4
def getDirVal(dir):
    if dir == DirType.top:
        val = 1
    if dir == DirType.bottom:
        val = 2
    if dir == DirType.left:
        val = 3
    if dir == DirType.right:
        val = 4
    return val
def cmpNeiDir(neidir1, neidir2):
    # print(neidir1)
    # print(neidir2)
    val1 = getDirVal(neidir1[1])
    val2 = getDirVal(neidir2[1])
    # print(val1)
    # print(val2)
    if val1 < val2:
        return -1
    elif val1 > val2:
        return 1
    else:
        return 0
    # return False


class Adj_Evaluator(object):
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

        self.stcConvertPath = STC2Path(self._ins, self._s_map)

        self._adjGraph = nx.Graph()
        for node in self._stcGraph.nodes():
            if self._stcGraph.nodes[node]['vert']._type == STCVertType.obstacle:
                continue
            self._adjGraph.add_node(node)

        self._indDic = dict()
        for ind, nodeInd in enumerate(self._adjGraph.nodes):
            self._indDic[ind] = nodeInd
        pass
        self._robStcIndLst = []
        for robID in range(self._robNum):
            stc_ind = self._s_map.gridInd2STCGridInd(GridInd(row=self._robPosLst[robID][0],
                                                      col = self._robPosLst[robID][1]))
            self._robStcIndLst.append(stc_ind)

    def evaluate(self,x):
        recordTimeBool = True
        if recordTimeBool:
            t_start = time.clock()
        self.generateSTree(x)
        if recordTimeBool:
            t_end = time.clock()
            print(' _runTime ', t_end - t_start)
    def generateSTree(self,x):

        self._adjGraph = nx.Graph()
        for node in self._stcGraph.nodes():
            if self._stcGraph.nodes[node]['vert']._type == STCVertType.obstacle:
                continue
            self._adjGraph.add_node(node)
        print(len(list(nx.connected_components(self._adjGraph))))


        findTimes = 0
        for x_ind, x_val in x:
            if findTimes > 200:
                break
            findTimes += 1
            nodeInd = self._indDic[x_ind]
            nodeInd = self._indDic[x_ind]
            neiLst = self._stcGraph.neighbors(nodeInd)
            dirNeiLst = []
            for nei in neiLst:
                # print(nei)
                nei_dir = getDir(nodeInd, nei)
                if self._adjGraph.has_edge(nodeInd,nei):
                    continue
                if nx.has_path(self._adjGraph, nodeInd, nei):
                    continue
                dirNeiLst.append((nei, nei_dir))
            # print('b', dirNeiLst)
            neiNum = len(dirNeiLst)
            if neiNum != 0:
                dirNeiLst = sorted(dirNeiLst, key=functools.cmp_to_key(cmpNeiDir))
                chsInd = floor(neiNum * x_val)
                self._adjGraph.add_edge(nodeInd, dirNeiLst[chsInd][0])
                # print('chsInd ', dirNeiLst[chsInd][0])
            else:
                # raise  Exception('xx')
                g_comp = list(nx.connected_components(self._adjGraph))
                for g_comp_ind, component in enumerate(g_comp):
                    if nodeInd in component:
                        break
                # if component
                # component method is not stable
                print(list(component))
                robNeiLst = []
                robCompBool = False
                if self.robComp(component):
                    robCompBool = True
                for stc_ind in component:
                    c_neiLst = self._stcGraph.neighbors(stc_ind)
                    for neiInd in c_neiLst:
                        if neiInd in component:
                            continue
                        # if nx.has_path(neiInd,)
                        if robCompBool:
                            if self.havePath2Rob(neiInd):
                                continue
                        dis = self.calNeiDis(nodeInd,neiInd)
                        robNeiLst.append((stc_ind, neiInd, dis))
                robNeiNum = len(robNeiLst)
                if robNeiNum != 0:
                    print('b ', robNeiLst)
                    robNeiSet = sorted(robNeiLst, key=lambda x: x[2])
                    print('a ', robNeiSet)
                    resInd = floor(robNeiNum*x_val)
                    self._adjGraph.add_edge(robNeiSet[resInd][0], robNeiSet[resInd][1])
                    print('sInd  =', robNeiSet[resInd][0], ' tInd = ', robNeiSet[resInd][1])
                    # exit()
                    # break
                print('len_comp = ', len(list(nx.connected_components(self._adjGraph))))
    def robComp(self,component):
        for robsctInd in self._robStcIndLst:
            if robsctInd in component:
                return True
        return False
    def havePath2Rob(self,stcInd):
        for robID in range(self._robNum):
            rob_stc_ind  = self._s_map.gridInd2STCGridInd(GridInd(row = self._robPosLst[robID][0],
                                                          col = self._robPosLst[robID][1]))
            if nx.has_path(self._adjGraph,rob_stc_ind, stcInd):
                return True
        return False


    def calNeiDis(self,cen_stc_ind, nei_stc_ind):
        vec_base = np.array([self._stcGraph.nodes[cen_stc_ind]['vert']._pos_x, self._stcGraph.nodes[cen_stc_ind]['vert']._pos_y])
        vec = np.array([self._stcGraph.nodes[nei_stc_ind]['vert']._pos_x, self._stcGraph.nodes[nei_stc_ind]['vert']._pos_y])
        dis = np.linalg.norm(vec_base -vec)
        return  dis

    def __str__(self):
        return str(self._ins) + '   adj_evaluator'

if __name__ == '__main__':
    ins = MCMPInstance()

    ins.loadCfg('D:\\py_code\\MCMP_encode\\benchmark\\r4_r50_c50_p0.8_s1000_Outdoor_Cfg.dat')

    adj_eval = Adj_Evaluator(ins)

    rdSeed = 1
    random.seed(rdSeed)

    num_nodes = adj_eval._adjGraph.number_of_nodes()
    np.random.seed(rdSeed)
    # rdPerm = np.random.permutation(num_nodes)
    # print('num_nodes = ', num_nodes)
    # print(rdPerm)
    # exit(rdPerm)
    # ind = []
    # for i in rdPerm:
    #     ind.append((i, np.random.random()))
    ind = []
    for i in range(num_nodes):
        ind.append((np.random.randint(num_nodes),np.random.random()))
    print('num_nodes = ', num_nodes)
    print('ind = ', ind)

    # exit()
    # for i in range(num_nodes):
    #     ind.append(random.random())
    # print(ind)
    adj_eval.evaluate(ind)
    exit()

    allEdgeLstPnt = []
    edgeLst = []
    for tInd,sInd in adj_eval._adjGraph.edges():
        t_pos_x = adj_eval._stcGraph.nodes[tInd]['vert']._pos_x
        t_pos_y = adj_eval._stcGraph.nodes[tInd]['vert']._pos_y
        s_pos_x = adj_eval._stcGraph.nodes[sInd]['vert']._pos_x
        s_pos_y = adj_eval._stcGraph.nodes[sInd]['vert']._pos_y
        edgeLst.append((t_pos_x, t_pos_y, s_pos_x, s_pos_y))
    allEdgeLstPnt.append(edgeLst)
    drawEvalSTCGraph(ins, edgePntLst=allEdgeLstPnt)


    exit()
    edgeLst = []
    _graph = adj_eval._stcGraph
    for edge in adj_eval._stcGraph.edges():
        # print(edge)
        sPnt_x = _graph.nodes[edge[0]]['vert']._pos_x
        sPnt_y = _graph.nodes[edge[0]]['vert']._pos_y
        tPnt_x = _graph.nodes[edge[1]]['vert']._pos_x
        tPnt_y = _graph.nodes[edge[1]]['vert']._pos_y
        edgeLst.append((sPnt_x, sPnt_y, tPnt_x, tPnt_y))
    #     # raise Exception('xx')
    drawSTCPic(ins, edgePntLst = edgeLst)
    print(adj_eval)
