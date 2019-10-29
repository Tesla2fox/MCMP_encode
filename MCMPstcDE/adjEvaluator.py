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

    def evaluate(self,x):
        self.generateSTree(x)

    def generateSTree(self,x):

        self._adjGraph = nx.Graph()
        for node in self._stcGraph.nodes():
            if self._stcGraph.nodes[node]['vert']._type == STCVertType.obstacle:
                continue
            self._adjGraph.add_node(node)
        # self._adjGraph.add_nodes_from(self._stcGraph.nodes())

        print(list(nx.connected_components(self._adjGraph)))
        # for robID in range(self._robNum):
        #     stc_ind  = self._s_map.gridInd2STCGridInd(GridInd(row = self._robPosLst[robID][0],
        #                                                   col = self._robPosLst[robID][1]))
            # self._adjGraph.a
        for x_ind, x_val in x:
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
            if neiNum == 0:
                continue
            dirNeiLst = sorted(dirNeiLst, key=functools.cmp_to_key(cmpNeiDir))
            chsInd = floor(neiNum * x_val)
            self._adjGraph.add_edge(nodeInd, dirNeiLst[chsInd][0])
        print(nx.is_connected(self._adjGraph))
        print(len(list(nx.connected_components(self._adjGraph))))

        # exit()
            # print(chsInd, dirNeiLst)
            # if nodeInd in self._adjGraph and dirNeiLst[chsInd][0] in self._adjGraph:
            #    pass
            # else:
        # x = []
        # self._edgeLst = []
        # ind_i = 0
        # for i,node in enumerate(self._stcGraph.nodes):
        #     neiLst = self._stcGraph.neighbors(node)
            # self._edgeLst.append((node, dirNeiLst[chsInd][0]))
            # print('a', dirNeiLst)
            # pass
            # exit()

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
    rdPerm = np.random.permutation(num_nodes)
    print('num_nodes = ', num_nodes)
    # print(rdPerm)
    # exit(rdPerm)
    ind = []
    for i in rdPerm:
        ind.append((i, np.random.random()))
    # for i in range(num_nodes):
    #     ind.append(random.random())
    # print(ind)
    adj_eval.evaluate(ind)
    # exit()

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
