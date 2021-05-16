#pragma once

#include "ThreadPool.hpp"

#include <vector>
#include <future>
#include <algorithm>
#include <random>

namespace crsGA
{

class UserData
{
  public:
    virtual ~UserData() {}
};

class IGen
{
  public:
    virtual ~IGen() {}
    virtual void mutate(const UserData *) = 0; // changes its state
    virtual void random(const UserData *) = 0; // random initialization
};

template <typename GenT>
class ComputeFitnessPolicy
{
  public:
    virtual ~ComputeFitnessPolicy() {}

    virtual float computeFitness(const std::vector<GenT> &, const UserData *) const = 0;
};

template <typename GenT,
          typename ComputeFitnessPolicyT>
class Chromosome : public ComputeFitnessPolicyT
{
  public:
    typedef GenT Gen;
    typedef ComputeFitnessPolicyT ComputeFitnessPolicy;
    typedef std::vector<Gen> GenesArray;

  protected:
    float _fitness;
    std::vector<Gen> _genes;

  public:
    Chromosome()
        : _fitness(0), _genes()
    {
    }
    virtual ~Chromosome() {}

    float getFitness() const { return _fitness; }
    float calculateFitness(const UserData *data)
    {
        _fitness = ComputeFitnessPolicyT::computeFitness(_genes, data);
        return _fitness;
    }
    void setNumGenes(uint32_t numGenes) { _genes.resize(numGenes); }
    size_t getNumGenes() const { return _genes.size(); }
    const GenT &getGen(size_t i) const { return _genes[i]; }
    GenT &getGen(size_t i) { return _genes[i]; }
    const GenesArray &getGenes() const { return _genes; }
    GenesArray &getGenes() { return _genes; }

    friend std::ostream &operator<<(std::ostream &os, const Chromosome &c)
    {
        for (size_t i = 0; i < c.getNumGenes(); ++i)
        {
            os << c.getGen(i);
        }
        return os;
    }
};

template <typename ChromosomeT>
class DefaultInitializationPolicy
{
  public:
    void initialize(std::vector<ChromosomeT> &chromosomes, const UserData *data) const
    {
        for (auto &c : chromosomes)
        {
            for (auto &gen : c.getGenes())
            {
                gen.random(data);
            }
        }
    }
};

template <typename ChromosomeT,
          typename InitializationPolicy = DefaultInitializationPolicy<ChromosomeT>>
class Population : public InitializationPolicy
{
  public:
    typedef ChromosomeT Chromosome;
    typedef std::vector<Chromosome> ChromosomeList;

  protected:
    ChromosomeList _chromosomes;
    mutable size_t _fittestChromosomeIndex;
    crsGA::ThreadPool _threadPool;
    uint32_t _minPerThread;

  public:
    Population(uint32_t populationSize, uint32_t numGenes)
        : _chromosomes(populationSize),
          _fittestChromosomeIndex(0),
          _threadPool(),
          _minPerThread(_chromosomes.size() / (_threadPool.getNumThreads() - 1))
    {
        for (auto &c : _chromosomes)
        {
            c.setNumGenes(numGenes);
        }
    }
    ~Population() {}

    void initializePopulation(const UserData *data = nullptr)
    {
        InitializationPolicy::initialize(_chromosomes, data);
    }

    void calculateFitnessBlock(typename ChromosomeList::iterator first, typename ChromosomeList::iterator last, const UserData *data)
    {
        for (auto it = first; it != last; ++it)
            it->calculateFitness(data);
    }

    template <typename Iterator>
    void parallelCalculateFitness(Iterator first, Iterator last, const UserData *data)
    {
        auto length = std::distance(first, last);
        if (length < _minPerThread)
            calculateFitnessBlock(first, last, data);
        const auto numThreads = _threadPool.getNumThreads();
        const size_t blockSize = length / numThreads;

        std::vector<std::future<void>> futures;
        Iterator blockStart = first;
        for (auto i = 0u; i < (numThreads - 1); ++i)
        {
            Iterator blockEnd = blockStart;
            std::advance(blockEnd, blockSize);
            futures.emplace_back(_threadPool.enqueue(&Population::calculateFitnessBlock, this, blockStart, blockEnd, data));
            blockStart = blockEnd;
        }
        calculateFitnessBlock(blockStart, last, data);
        for (auto i = 0u; i < (numThreads - 1); ++i)
        {
            futures[i].wait();
        }
    }

    void calculateFitness(const UserData *data = nullptr)
    {
        parallelCalculateFitness(_chromosomes.begin(), _chromosomes.end(), data);
        //calculateFitnessBlock(_chromosomes.begin(), _chromosomes.end(), data);
        sortByFitness();
        findFittestChromosome();
    }

    void mutateBlock(double mutationFactor, typename ChromosomeList::iterator first, typename ChromosomeList::iterator last, const UserData *data)
    {
        std::random_device rd;                             // used once to initialise (seed) engine
        std::mt19937 rng(rd());                            // random-number engine used (Mersenne-Twister in this case)
        std::uniform_real_distribution<float> uni(0, 1.0); // guaranteed unbiased
        for (auto it = first; it != last; ++it)
        {
            //Select if mutate this chromosome
            for (auto &gen : it->getGenes())
            {
                if (uni(rng) > mutationFactor) // random mutation
                {
                    gen.mutate(data);
                }
            }
        }
    }

    void mutate(double mutationFactor, const UserData *data)
    {
        mutateBlock(mutationFactor, _chromosomes.begin(), _chromosomes.end(), data);
    }

    const Chromosome &getFittestChromosome() const
    {
        return _chromosomes[_fittestChromosomeIndex];
    }

    const Chromosome &findFittestChromosome() const
    {
        auto result = std::min_element(_chromosomes.begin(), _chromosomes.end(),
                                       [](const Chromosome &c1, const Chromosome &c2) {
                                           LOG_DEBUG("C1 Fitness:", c1.getFitness(), " C2 Fitness:", c2.getFitness());
                                           return (c1.getFitness() < c2.getFitness());
                                       });
        _fittestChromosomeIndex = std::distance(_chromosomes.begin(), result);
        return *result;
    }
    const Chromosome &findSecondFittestChromosome() const
    {
        auto first = _chromosomes.begin();
        auto last = _chromosomes.end();
        if (first == last)
        {
            return *first;
        }

        auto fittest = first;
        auto runner_up = fittest;

        for (++first; first != last; ++first)
        {
            if ((*runner_up).getFitness() >= (*first).getFitness())
            {
                runner_up = first;
                if ((*fittest).getFitness() >= (*runner_up).getFitness())
                {
                    std::swap(fittest, runner_up);
                }
            }
        }

        return *runner_up;
    }
    void sortByFitness()
    {
        std::sort(_chromosomes.begin(), _chromosomes.end(), [](const Chromosome &c1, const Chromosome &c2) {
            return c1.getFitness() < c2.getFitness();
        });
    }

    const std::vector<Chromosome> &getChromosomes() const
    {
        return _chromosomes;
    }
    std::vector<Chromosome> &getChromosomes()
    {
        return _chromosomes;
    }

    const Chromosome &getChromosome(size_t index) const
    {
        return _chromosomes[index];
    }
    Chromosome &getChromosome(size_t index)
    {
        return _chromosomes[index];
    }

    void setChromosome(size_t index, const Chromosome &chromosome)
    {
        _chromosomes[index] = chromosome;
    }
};

template <typename ChromosomeT,
          typename PopulationT>
class DefaultSelectionPolicy
{
  public:
    std::vector<ChromosomeT> select(const PopulationT &population) const
    {
        std::vector<ChromosomeT> fittestList;
        fittestList.push_back(population.getChromosome(0));
        fittestList.push_back(population.findSecondFittestChromosome());
        return fittestList;
    }
};

template <typename ChromosomeT>
class DefaultCrossoverPolicy
{
  public:
    std::vector<ChromosomeT> crossover(const std::vector<ChromosomeT> &parents) const
    {
        std::vector<ChromosomeT> children(parents);
        //Select a random crossover point
        std::random_device rd;                                                       // used once to initialise (seed) engine
        std::mt19937 rng(rd());                                                      // random-number engine used (Mersenne-Twister in this case)
        std::uniform_int_distribution<size_t> uni(0u, parents[0].getNumGenes() - 1); // guaranteed unbiased
        auto crossOverPoint = uni(rng);
        std::swap_ranges(children[0].getGenes().begin(),
                         children[0].getGenes().begin() + crossOverPoint,
                         children[1].getGenes().begin());
        crossOverPoint = uni(rng);
        std::swap_ranges(children[1].getGenes().begin(),
                         children[1].getGenes().begin() + crossOverPoint,
                         children[0].getGenes().begin());
        return children;
    }
};

template <typename Chromosome,
          typename Population>
class DefaultReplacementPolicy
{
  public:
    std::vector<size_t> findLeastFittestIndices(const Population &population) const
    {
        const auto &chromosomes = population.getChromosomes();
        auto result = std::max_element(chromosomes.begin(), chromosomes.end(),
                                       [](const Chromosome &c1, const Chromosome &c2) {
                                           return (c1.getFitness() < c2.getFitness());
                                       });
        std::vector<size_t> leastFittestIndices;
        leastFittestIndices.push_back(std::distance(chromosomes.begin(), result));
        return leastFittestIndices;
    }
};
} // namespace ait