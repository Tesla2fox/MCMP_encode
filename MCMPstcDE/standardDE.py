
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
from MCMPstcDE.evaluator import STCEvaluator
import  sys



import random
import array

import numpy

from deap import base
from deap import benchmarks
from deap import creator
from deap import tools



def stcEval(individual,stc_eval :STCEvaluator):
    robPatternLst = [[] for x in range(stc_eval._robNum)]
    robID = 0
    for i in range(0,len(individual),3):
    # for x in individual:
        robPatternLst[robID].append((individual[i],individual[i + 1],individual[i + 2]))
        # print(robPatternLst)
        if robID == (stc_eval._robNum - 1):
            robID = 0
        else:
            robID += 1
    # print(robPatternLst)
    invalid, fitness = stc_eval.evaluate(robPatternLst)
    return fitness,


def fixGene(_x):
    if _x < 0:
        return 1 + _x
    if _x > 1 :
        return _x - 1
    return _x

def initOperator(stc_eval :STCEvaluator):
    # Problem dimension

    creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
    creator.create("Individual", list, fitness=creator.FitnessMin)

    toolbox = base.Toolbox()
    toolbox.register("attr_float", random.random)
    toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.attr_float, stc_eval._stcGraph.number_of_nodes())
    toolbox.register("population", tools.initRepeat, list, toolbox.individual)
    toolbox.register("select", tools.selRandom, k=3)
    toolbox.register("evaluate", stcEval, stc_eval = stc_eval)
    # creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
    # creator.create("Individual", list)
    # toolbox = base.Toolbox()
    # toolbox.register("attr_float", random.random)
    # toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.attr_float, 3)
    # toolbox.register("population", tools.initRepeat, list, toolbox.individual)
    # toolbox.register("mutate", mutDE, f=0.8)
    # toolbox.register("select", tools.selRandom, k=3)
    # toolbox.register("mate", cxExponential, cr=0.8)
    # toolbox.register("fixGene",)
    print('init operator')
    return toolbox



if __name__ == "__main__":

    print('begin the standard DE method')
    ins = MCMPInstance()
    confDir = 'D:\\pycode\\MCMP_encode\\benchmark\\'
    if len(sys.argv) == 4:
        fileName = sys.argv[1]
        baseSeed = int(sys.argv[2])
        maxSampleTimes = int(sys.argv[3])
        ins.loadCfg(confDir + fileName + '.dat')
        # random.seed(seed)
        maxRunTimes = int(sys.argv[3])
    else:
        print('pycharm run')
        fileName = 'r2_r40_c20_p0.9_s1000_Outdoor_Cfg'
        baseSeed = 0
        ins.loadCfg(confDir + fileName + '.dat')
        maxRunTimes = 1


    stc_eval = STCEvaluator(ins)
    toolbox = initOperator(stc_eval)
    for seed in range(maxRunTimes):
        rSeed = baseSeed + seed
        random.seed(rSeed)

        fileCfg  = 'D:\\pycode\\MCMP_encode//resData//' +fileName+ '//stdde//res_s' + str(rSeed)  + '.dat'
        f_con = open(fileCfg, 'w')

        CR = 0.25
        F = 1
        MU = 300
        NGEN = 200

        pop = toolbox.population(n=MU);
        hof = tools.HallOfFame(1)
        stats = tools.Statistics(lambda ind: ind.fitness.values)
        stats.register("avg", numpy.mean)
        stats.register("std", numpy.std)
        stats.register("min", numpy.min)
        stats.register("max", numpy.max)

        logbook = tools.Logbook()
        logbook.header = "gen", "evals", "std", "min", "avg", "max"

        # Evaluate the individuals
        fitnesses = toolbox.map(toolbox.evaluate, pop)
        for ind, fit in zip(pop, fitnesses):
            ind.fitness.values = fit

        record = stats.compile(pop)
        logbook.record(gen=0, evals=len(pop), **record)
        print(logbook.stream)
        # print(record)
        f_con.write(str(record['min']) + '\n')
        f_con.flush()

        for g in range(1, NGEN):
            for k, agent in enumerate(pop):
                a, b, c = toolbox.select(pop)
                y = toolbox.clone(agent)
                index = random.randrange(stc_eval._stcGraph.number_of_nodes())
                for i, value in enumerate(agent):
                    if i == index or random.random() < CR:
                        y[i] = a[i] + F * (b[i] - c[i])
                        if not ( 0 <= y[i] <= 1):
                            y[i] = fixGene(y[i])
                        # if fixGene(y[i]):
                y.fitness.values = toolbox.evaluate(y)
                if y.fitness > agent.fitness:
                    pop[k] = y
            hof.update(pop)
            record = stats.compile(pop)
            logbook.record(gen=g, evals=len(pop), **record)
            f_con.write(str(record['min']) + '\n')
            f_con.flush()
            print(logbook.stream)

        print('hof =', hof)
        print("Best individual is ", hof[0], hof[0].fitness.values[0])


