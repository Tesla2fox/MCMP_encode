import random
import array
import numpy

from deap import base
from deap import creator
from deap import tools
from itertools import chain


# from deap import  benchmarks

def mutDE(y, a, b, c, f):
    size = len(y)
    for i in range(len(y)):
        y[i] = a[i] + f*(b[i]-c[i])
    return y

def cxBinomial(x, y, cr):
    size = len(x)
    index = random.randrange(size)
    for i in range(size):
        if i == index or random.random() < cr:
            x[i] = y[i]
    return x

def cxExponential(x, y, cr):
    size = len(x)
    index = random.randrange(size)
    # Loop on the indices index -> end, then on 0 -> index
    for i in chain(range(index, size), range(0, index)):
        x[i] = y[i]
        if random.random() < cr:
            break
    return x

# def fixGene(x):
#
#     if not (0<= x[0] <= 1):
#         raise  Exception('XX')
#     if not (0<= x[0] <= 1)


def initOperator():
    # creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
    creator.create("Individual", array.array, typecode='d')
    toolbox = base.Toolbox()
    toolbox.register("attr_float", random.random)
    toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.attr_float, 3)
    toolbox.register("population", tools.initRepeat, list, toolbox.individual)
    toolbox.register("mutate", mutDE, f=0.8)
    toolbox.register("select", tools.selRandom, k=3)
    toolbox.register("mate", cxExponential, cr=0.8)
    # toolbox.register("fixGene",)
    print('init operator')
    return toolbox

def getOffPop(pop,toolbox):
    offPop = []
    for k,agent in enumerate(pop):
        # print(k)
        # print(agent)
        a,b,c = toolbox.select(pop)
        a,b,c = [toolbox.clone(ind) for ind in toolbox.select(pop)]
        # print(a,b,c)
        x = toolbox.clone(agent)
        y = toolbox.clone(agent)
        y = toolbox.mutate(y,a,b,c)
        z = toolbox.mate(x,y)
        # z = toolbox.mate(x, y)
        if not (0<= z[1]<= 1):
            z[1] = random.random()
        if not (0<= z[0] <= 1):
            z[0] = random.random()
        offPop.append(z)
        # print(y)
    return offPop
        # index =

