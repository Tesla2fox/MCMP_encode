from deap import base
from deap import creator
from deap import tools
import  numpy as np
from deap import algorithms

import  MCMPdecode
from MCMPdecode import  MCMPInstance,MCMP_Decode

creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
creator.create("Individual", list, typecode='i', fitness=creator.FitnessMin)

toolbox = base.Toolbox()


np.random.seed(20)

ins = MCMPInstance()
ins.loadCfg('.\\benchmark\\r2_r40_c20_p0.9_s1000_Outdoor_Cfg.dat')

PATTREN_NUM = 8
ROB_NUM = 2
CODE_NUM = 30
MAX_ROW = ins._row
MAX_COL = ins._col

def mcmp_init_encode(patternNum,robNum,codeNum,maxRow,maxCol):
    typeLst = np.random.randint(0, patternNum, size=robNum+ codeNum)
    stepLst = np.random.randint(1,200,size=robNum  + codeNum)
    rowLst = np.random.randint(0, maxRow, size = codeNum)
    colLst = np.random.randint(0, maxCol, size = codeNum)
    lvl = np.random.randint(0,20,size = codeNum)

    randChrom = [typeLst[0],stepLst[0],typeLst[1],stepLst[1]]

    for i in range(codeNum):
        randChrom.append(rowLst[i])
        randChrom.append(colLst[i])
        randChrom.append(typeLst[i + robNum])
        randChrom.append(stepLst[i + robNum])
        randChrom.append(lvl[i])
    return  randChrom
    # print('randChrom = ', randChrom)



mcmp_decode = MCMP_Decode(ins)

def mcmp_eval_encode(individual):
    return mcmp_decode.decode(individual),



def mcmp_mate(ind1, ind2, indpb):
    """Swaps the number of perticular items between two individuals"""
    for i in range(len(ind1)):
        if np.random.random() < indpb:
            ind1[i], ind2[i] = ind2[i], ind1[i]
    return ind1, ind2

def mcmp_mutate(individual, indpb):
    # print(individual)
    for i in range(len(individual)):
        if i == 0 or i == 2 or ((i-4) % 5 == 0 and i >=4):
            individual[i] = np.random.randint(0,8)
        if i == 1 or i == 3 or ((i-4) %5 == 1 and i >= 4):
            individual[i] = np.random.randint(1,200)
        if (i-4) % 5 == 4 and i >=4:
            individual[i] = np.random.randint(1,20)
    # print(individual)
    return individual,

# indice +



toolbox.register("mcmp_attr",mcmp_init_encode,PATTREN_NUM,ROB_NUM,CODE_NUM,MAX_ROW,MAX_COL)
toolbox.register("individual", tools.initIterate, creator.Individual,
                 toolbox.mcmp_attr)

# define the population to be a list of individuals
toolbox.register("population", tools.initRepeat, list, toolbox.individual)

# ----------
# Operator registration
# ----------
# register the goal / fitness function
toolbox.register("evaluate",mcmp_eval_encode)

# register the crossover operator
toolbox.register("mate",mcmp_mate, indpb = 0.5)

# register a mutation operator with a probability to
# flip each attribute/gene of 0.05
toolbox.register("mutate", mcmp_mutate, indpb=0.01)
# tools.mutShuffleIndexes
# tools.mutShuffleIndexes()

# operator for selecting individuals for breeding the next
# generation: each individual of the current generation
# is replaced by the 'fittest' (best) of three individuals
# drawn randomly from the current generation.
# tools.selAutomaticEpsilonLexicase(), tournsize=3
# toolbox.register("select",tools.selTournament,)
toolbox.register("select", tools.selTournament, tournsize = 3)
# toolbox.register("select", tools.selAutomaticEpsilonLexicase, 3)


if __name__ == '__main__':

    f_data = open('.//debugData//GA_test_2.dat' , 'w')

    pop = toolbox.population(n = 300)
    CXPB = 0.5
    MUTPB = 0.2

    print("Start of evolution")
    np.random.seed(20)
    # Evaluate the entire population
    fitnesses = list(map(toolbox.evaluate, pop))
    # raise Exception('xx')
    for ind, fit in zip(pop, fitnesses):
        # print('xx')
        ind.fitness.values = fit

    print("  Evaluated %i individuals" % len(pop))

    # Extracting all the fitnesses of
    fits = [ind.fitness.values[0] for ind in pop]

    # Variable keeping track of the number of generations
    g = 0


    # Begin the evolution
    while g < 2:
        # A new generation
        g = g + 1
        print("-- Generation %i --" % g)

        # Select the next generation individuals
        offspring = toolbox.select(pop, len(pop))
        # Clone the selected individuals
        offspring = list(map(toolbox.clone, offspring))

        # Apply crossover and mutation on the offspring
        for child1, child2 in zip(offspring[::2], offspring[1::2]):

            # toolbox.mate(child1,child2)
            # cross two individuals with probability CXPB
            if np.random.random() < CXPB:
                toolbox.mate(child1, child2)
                # fitness values of the children
                # must be recalculated later
                del child1.fitness.values
                del child2.fitness.values

        for mutant in offspring:
            # toolbox.mutate(mutant)
            # mutate an individual with probability MUTPB
            if np.random.random() < MUTPB:
                toolbox.mutate(mutant)
                del mutant.fitness.values

        # Evaluate the individuals with an invalid fitness
        invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
        fitnesses = map(toolbox.evaluate, invalid_ind)
        for ind, fit in zip(invalid_ind, fitnesses):
            ind.fitness.values = fit

        print("  Evaluated %i individuals" % len(invalid_ind))

        # The population is entirely replaced by the offspring
        pop[:] = offspring

        # Gather all the fitnesses in one list and print the stats
        fits = [ind.fitness.values[0] for ind in pop]


        length = len(pop)
        mean = sum(fits) / length
        sum2 = sum(x * x for x in fits)
        std = abs(sum2 / length - mean ** 2) ** 0.5

        print("  Min %s" % min(fits))
        print("  Max %s" % max(fits))
        print("  Avg %s" % mean)
        print("  Std %s" % std)

        f_data.write(str(g)+' '+ str(min(fits))+ ' ' + str(max(fits)) + ' ' + str(mean)+' ' + str(std)+'\n')
        f_data.flush()

    print("-- End of (successful) evolution --")