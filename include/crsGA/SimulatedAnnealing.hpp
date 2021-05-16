#pragma once

#include "ThreadPool.hpp"
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
#include <chrono>

namespace crsGA
{

template <typename GenT,
          typename ChromosomeT,
          typename PopulationT,
          typename SelectionPolicy = DefaultSelectionPolicy<ChromosomeT, PopulationT>>
class SimulatedAnnealing : public SelectionPolicy
{
  public:
    typedef GenT Gen;
    typedef ChromosomeT Chromosome;
    typedef PopulationT Population;

  protected:
    uint32_t _iteration;
    Population _population;
    Chromosome _currentSolution;
    Chromosome _bestSolution;
    float _mutationFactor;
    float _minFitness; // The algorithm will find the min of all of this values
    float _initialTemperature;
    float _temperature;
    float _k;
    std::shared_ptr<UserData> _data;

  public:
    SimulatedAnnealing(uint32_t populationSize = 100, uint32_t numGenes = 6, float temperature = 100.0, float minFitness = 0.001f)
        : _iteration(0), _population(populationSize, numGenes),
          _currentSolution(), _bestSolution(), _mutationFactor(50.0f), _minFitness(minFitness),
          _initialTemperature(temperature), _temperature(_initialTemperature), _k(0.01), _data()
    {
    }
    ~SimulatedAnnealing()
    {
    }

    void setUserData(const std::shared_ptr<UserData> &data)
    {
        _data = data;
    }

    const Population &getPopulation() const { return _population; }

    size_t getGeneration() const { return _iteration; }

    Chromosome mutation(const Chromosome &c) const
    {
        //Select a random crossover point
        std::random_device rd;                             // used once to initialise (seed) engine
        std::mt19937 rng(rd());                            // random-number engine used (Mersenne-Twister in this case)
        std::uniform_real_distribution<float> uni(0, 1.0); // guaranteed unbiased
        auto newSolution = c;
        for (auto &gen : newSolution.getGenes())
        {
            if (uni(rng) > _mutationFactor) // random mutation
            {
                gen.mutate(_data.get());
            }
        }
        newSolution.calculateFitness(_data.get());
        return newSolution;
    }

    bool isSolution(const Chromosome &c) const
    {
        return (c.getFitness() < _minFitness) || _temperature < 0.01;
    }

    void reset()
    {
        _population.initializePopulation(_data.get());
        _population.calculateFitness(_data.get());
        auto initialSolution = SelectionPolicy::select(_population)[0];
        _currentSolution = initialSolution;
        _bestSolution = _currentSolution;
        _temperature = _initialTemperature;
        LOG_WARNING("Fitness=", _bestSolution.getFitness(), "Temperature:", _temperature, " Chromosome: {", _bestSolution, "}");
    }

    double random_probability() const
    {
        //Select a random crossover point
        std::random_device rd;                             // used once to initialise (seed) engine
        std::mt19937 rng(rd());                            // random-number engine used (Mersenne-Twister in this case)
        std::uniform_real_distribution<float> uni(0, 1.0); // guaranteed unbiased
        return uni(rng);
    }

    const Chromosome &getFittestChromosome() const
    {
        return _bestSolution;
    }

    void step(float simulationTime)
    {
        if (isSolution(_bestSolution))
        {
            return;
        }
        _iteration++;

        /*if (_temperature < 5)
        {
            _population.initializePopulation(_data.get());
            _population.calculateFitness(_data.get());
            //_population.setChromosome(_population.getChromosomes().size() - 1, _bestSolution);
            //_population.mutate(_mutationFactor, _data.get());
            auto initialSolution = SelectionPolicy::select(_population)[0];
            _currentSolution = initialSolution;
            _temperature = _initialTemperature;
        }*/

        //auto newSolution = SelectionPolicy::select(_population)[0]; // random selection of a solution
        auto newSolution = mutation(_currentSolution);
        auto fitnessVariation = newSolution.getFitness() - _currentSolution.getFitness();
        if (fitnessVariation < 0)
        {
            _currentSolution = newSolution;
            if (newSolution.getFitness() < _bestSolution.getFitness())
            {
                _bestSolution = newSolution;
            }
        }
        else
        {
            if (std::exp(-fitnessVariation / _temperature) > random_probability())
            {
                _currentSolution = newSolution;
                _temperature = _temperature / (1 + _k * _temperature);
            }
        }
        std::cout << simulationTime << "; " << _bestSolution.getFitness() << "; " << _temperature << std::endl;
        if (isSolution(_bestSolution))
        {
            LOG_WARNING("Solution found in iteration ", _iteration);
            LOG_WARNING("Fitness=", _population.getFittestChromosome().getFitness(), " Chromosome: {", _population.getFittestChromosome(), "}");
        }
    }

    void run(double maxDuration, bool resetPopulation = false)
    {
        if (resetPopulation)
            reset();
        std::cout << "Time"
                  << ";"
                  << "Fitness"
                  << ";"
                  << "Temperature" << std::endl;
        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

        while (!isSolution(_bestSolution))
        {
            std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
            auto simulationTime = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
            std::cout << simulationTime << ";" << _bestSolution.getFitness() << ";" << _temperature << std::endl;
            step(simulationTime);
            if (simulationTime >= maxDuration)
            {
                LOG_WARNING("Max duration reached");
                break;
            }
        };
    }

    void setMutationFactor(float mutationFactor) { _mutationFactor = mutationFactor; }
    void setTemperatureFactor(float k) { _k = k; }
};
} // namespace crsGA