


import os,sys
AbsolutePath = os.path.abspath(__file__)
#将相对路径转换成绝对路径
SuperiorCatalogue = os.path.dirname(AbsolutePath)
#相对路径的上级路径
# print(AbsolutePath)
# print(SuperiorCatalogue)
BaseDir = os.path.dirname(SuperiorCatalogue)
# print(BaseDir)
#在“SuperiorCatalogue”的基础上在脱掉一层路径，得到我们想要的路径。
if BaseDir in sys.path:
    pass
else:
    sys.path.append(BaseDir)


from MCMPinstance import MCMPInstance
import random
import copy
from deap import base
from deap import creator
from deap import tools
from MCMPstcDE.operaterDE import  initOperator,getOffPop
from enum import Enum
from MCMPstcDE.evaluator import STCEvaluator

class updateType(Enum):
    insert = 0
    replace = 1
    delete = 2

def insertPop(pop,ind):
    insert_pop = copy.copy(pop)
    insert_pos = random.randint(0, len(pop)-1)
    insert_pop.insert(insert_pos,ind)
    return insert_pop

def replacePop(pop,ind):
    replace_pop = copy.copy(pop)
    replace_pos = random.randint(0, len(pop)-1)
    replace_pop[replace_pos] = ind
    return replace_pop

def deletePop(pop):
    del_pop = copy.copy(pop)
    del_pos = random.randint(0, len(pop)-1)
    del del_pop[del_pos]
    return del_pop

if __name__ == '__main__':
    print(sys.argv)
    ins = MCMPInstance()
    confDir = 'D:\\pycode\\MCMP_encode\\benchmark\\'
    if len(sys.argv) == 4:
        fileName = sys.argv[1]
        baseSeed = int(sys.argv[2])
        ins.loadCfg(confDir + fileName + '.dat')
        # random.seed(seed)
        runTimes  = int(sys.argv[3])
    else:
        print('pycharm run')
        fileName = 'r2_r40_c20_p0.9_s1000_Outdoor_Cfg'
        ins.loadCfg(confDir + fileName + '.dat')
        seed = 0
        runTimes = 1
    # ins.loadCfg('D:\\py_code\\MCMP_encode\\benchmark\\r2_r20_c20_p0.9_s1000_Outdoor_Cfg.dat')
    stc_eval = STCEvaluator(ins)
    for seed in range(runTimes):
        rSeed = baseSeed  + seed
        random.seed(rSeed)
        # fileCfg = './/resData//r' + str(robNum) + '_r' + str(row) + '_c' + str(col) + '_p' + str(p[0]) + '_s' + str(
        #     r_seed) + '_Outdoor_Cfg.dat'
        # f_con = open(fileCfg, 'w')
        fileCfg  = './/resData//' +fileName+ '//res_devip_s' + str(rSeed)  + '.dat'
        f_con = open(fileCfg, 'w')
        print('seed = ',rSeed)
        toolbox = initOperator()
        pop = toolbox.population(100)
        fitBool, fitness = stc_eval.evaluate(pop)
        # print(pop)
        maxEvaluationTimes = 20
        evaluationTimes = 0
        endBool = False
        while True:
            subPop = getOffPop(pop, toolbox)
            for k,ind in enumerate(subPop):
                insPop = insertPop(pop,ind)
                # print(insPop)
                insBool, insFitness = stc_eval.evaluate(insPop)
                repPop = replacePop(pop,ind)
                # print(repPop)
                repBool, repFitness = stc_eval.evaluate(repPop)
                delPop = deletePop(pop)
                # print(delPop)
                delBool, delFitness = stc_eval.evaluate(delPop)
                fitnessLst = [insFitness - fitness, repFitness - fitness, delFitness - fitness]
                minFitness = min(fitnessLst)
                if minFitness < 0:
                    minIndex  = fitnessLst.index(minFitness)
                    # fitness = minFitness
                    if minIndex == 0:
                        pop = insPop
                        fitness = insFitness
                    elif minIndex == 1:
                        pop = repPop
                        fitness = repFitness
                    else:
                        pop = delPop
                        fitness = delFitness
                evaluationTimes += 3
                print(fitness)
                f_con.write(str(fitness) + '\n')
                f_con.flush()
                if evaluationTimes > maxEvaluationTimes:
                    endBool  = True
                    minPop = pop
                    break
            if endBool:
                break
        # print('ins',insPop)
        # print('rep',repPop)
        # print('del',delPop)
        # exit()
    # print(subPop)
    print(minPop)
