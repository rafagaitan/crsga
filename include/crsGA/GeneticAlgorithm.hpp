#pragma once

#include "Export.hpp"
#include "Logger.hpp"
#include "Common.hpp"

#include <cstdint>
#include <cstddef>
#include <vector>
#include <random>
#include <algorithm>
#include <string>
#include <memory>
#include <future>
#include <iostream>

namespace crsGA
{
template <typename GenT,
          typename ChromosomeT,
          typename PopulationT,
          typename SelectionPolicy = DefaultSelectionPolicy<ChromosomeT, PopulationT>,
          typename CrossoverPolicy = DefaultCrossoverPolicy<ChromosomeT>,
          typename ReplacementPolicy = DefaultReplacementPolicy<ChromosomeT, PopulationT>>
class GeneticAlgorithm : public SelectionPolicy, public CrossoverPolicy, public ReplacementPolicy
{
  public:
    typedef GenT Gen;
    typedef ChromosomeT Chromosome;
    typedef PopulationT Population;

  protected:
    uint32_t _maxGenerations;
    uint32_t _generation;
    Population _population;
    std::vector<Chromosome> _fittestChromosomes;
    float _minFitness; // The algorithm will find the min of all of this values
    float _mutationFactor;
    std::shared_ptr<UserData> _data;

  public:
    GeneticAlgorithm(uint32_t populationSize = 100, uint32_t numGenes = 6, float minFitness = 0.001f)
        : _generation(0), _population(populationSize, numGenes), _fittestChromosomes(), _minFitness(minFitness), _mutationFactor(10)
    {
    }
    ~GeneticAlgorithm()
    {
    }

    void setUserData(const std::shared_ptr<UserData> &data)
    {
        _data = data;
    }

    size_t getGeneration() const { return _generation; }

    const Population &getPopulation() const { return _population; }

    //Selection
    void selection()
    {
        //Select the most fittest individual
        _fittestChromosomes = SelectionPolicy::select(_population);
    }

    //Crossover
    void crossover()
    {
        _fittestChromosomes = CrossoverPolicy::crossover(_fittestChromosomes);
    }

    //Mutation
    void mutation()
    {
        //Select a random crossover point
        std::random_device rd;                             // used once to initialise (seed) engine
        std::mt19937 rng(rd());                            // random-number engine used (Mersenne-Twister in this case)
        std::uniform_real_distribution<float> uni(0, 1.0); // guaranteed unbiased

        for (auto &c : _fittestChromosomes)
        {
            //Select if mutate this chromosome
            for (auto &gen : c.getGenes())
            {
                if (uni(rng) > _mutationFactor) // random mutation
                {
                    gen.mutate(_data.get());
                }
            }
        }

        //_population.mutate(_mutationFactor, _data.get());
    }

    //Get fittest offspring
    size_t findFittestOffspringIndex(const std::vector<Chromosome> &chromosomes) const
    {
        auto result = std::min_element(chromosomes.begin(), chromosomes.end(),
                                       [](const Chromosome &c1, const Chromosome &c2) {
                                           return (c1.getFitness() < c2.getFitness());
                                       });
        return std::distance(chromosomes.begin(), result);
    }

    //Replace least fittest individuals from most fittest offsprings
    void addFittestOffspring()
    {
        //Update fitness values of offspring
        for (auto &c : _fittestChromosomes)
        {
            c.calculateFitness(_data.get());
        }

        //Get index of least fit individual
        auto leastFittestIndicesList = ReplacementPolicy::findLeastFittestIndices(_population);

        //Replace least fittest individual from most fittest offspring
        auto localFittestChromosomes = _fittestChromosomes;
        for (auto i = 0u; i < leastFittestIndicesList.size(); i++)
        {
            auto fittestChromosomeIndex = findFittestOffspringIndex(localFittestChromosomes);
            _population.setChromosome(leastFittestIndicesList[i], localFittestChromosomes[fittestChromosomeIndex]);
            localFittestChromosomes.erase(localFittestChromosomes.begin() + fittestChromosomeIndex);
        }
    }

    bool isSolution(const Chromosome &c) const
    {
        return (c.getFitness() <= _minFitness);
    }

    void reset()
    {
        _population.initializePopulation(_data.get());
        _population.calculateFitness(_data.get());
        LOG_DEBUG("Fitness=", _population.getFittestChromosome().getFitness(), " Chromosome: {", _population.getFittestChromosome(), "}");
    }

    void step(float /*simulationTime*/)
    {
        if (isSolution(_population.getFittestChromosome()))
        {
            return;
        }
        _generation++;

        selection();
        crossover();
        mutation(); // should be random
        addFittestOffspring();
        _population.calculateFitness(_data.get());

        //if ((simulationTime-(int)simulationTime) <= 0.000001)
        //    LOG_DEBUG("Fitness=", _population.getFittestChromosome().getFitness(), " Chromosome: {", _population.getFittestChromosome(), "}");

        if (isSolution(_population.getFittestChromosome()))
        {
            LOG_DEBUG("Solution found in generation ", _generation);
            LOG_DEBUG("Fitness=", _population.getFittestChromosome().getFitness(), " Chromosome: {", _population.getFittestChromosome(), "}");
        }
    }

    void run(double maxDuration, bool resetPopulation = false)
    {
        if (resetPopulation)
            reset();
        std::cout << "Time"
                  << "\t"
                  << "Fitness" << std::endl;
        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
        auto simulationTime = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t1).count();
        while (!isSolution(_population.getFittestChromosome()))
        {
            std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
            simulationTime = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
            std::cout << simulationTime << "\t" << _population.getFittestChromosome().getFitness() << std::endl;
            step(simulationTime);
            if (simulationTime >= maxDuration)
            {
                LOG_WARNING("Max duration reached");
                break;
            }
        }
        std::cout << simulationTime << "\t" << _population.getFittestChromosome().getFitness() << std::endl;
        std::cout << "Best:" << _population.getFittestChromosome() << std::endl;
    }

    void setMutationFactor(float mutationFactor) { _mutationFactor = mutationFactor; }
};
} // namespace ait
