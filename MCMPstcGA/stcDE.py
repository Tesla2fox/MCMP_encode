
from MCMPinstance import MCMPInstance
import random
import copy
from deap import base
from deap import creator
from deap import tools
from MCMPstcGA.operaterDE import  initOperator,getOffPop
from enum import Enum
from MCMPstcGA.evaluator import STCEvaluator

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
    print('xxx')

    ins = MCMPInstance()
    ins.loadCfg('D:\\pycode\\MCMP_encode\\benchmark\\r2_r40_c20_p0.9_s1000_Outdoor_Cfg.dat')
    # ins.loadCfg('D:\\py_code\\MCMP_encode\\benchmark\\r2_r20_c20_p0.9_s1000_Outdoor_Cfg.dat')
    stc_eval = STCEvaluator(ins)
    # random.seed(1)
    toolbox = initOperator()
    pop = toolbox.population(100)
    fitBool, fitness = stc_eval.evaluate(pop)
    print(pop)
    maxEvaluationTimes = 2000
    evaluationTimes = 0
    endBool = False
    while True:
        subPop = getOffPop(pop, toolbox)
        for k,ind in enumerate(subPop):
            insPop = insertPop(pop,ind)
            insBool, insFitness = stc_eval.evaluate(insPop)
            repPop = replacePop(pop,ind)
            repBool, repFitness = stc_eval.evaluate(repPop)
            delPop = deletePop(pop)
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
