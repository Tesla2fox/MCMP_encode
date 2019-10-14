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


def initOperator():
    # creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
    creator.create("Individual", array.array, typecode='d')

    toolbox = base.Toolbox()
    toolbox.register("attr_float", random.uniform, -3, 3)
    toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.attr_float, 2)
    toolbox.register("population", tools.initRepeat, list, toolbox.individual)
    toolbox.register("mutate", mutDE, f=0.8)
    toolbox.register("select", tools.selRandom, k=3)
    toolbox.register("mate", cxExponential, cr=0.8)

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
        offPop.append(z)
        # print(y)

    return offPop
        # index =

