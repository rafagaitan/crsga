#include <crsGA/GeneticAlgorithm.hpp>

#include <random>

float computeProbability()
{
    std::random_device rd;
    std::uniform_real_distribution<float> distribution(0.0, 100.0);
    std::mt19937 engine(rd()); // Mersenne twister MT19937

    return distribution(engine);
}

/**
 * @brief Represents the smallest piece of information.
 * 
 * A bit of the bitstring of the onemax algorithm
 * 
 */
class Gen : public crsGA::IGen
{
  public:
    // Gen data (1 bit)
    uint8_t bit = 0;

  public:
    Gen() = default;
    virtual ~Gen() = default;
    // How to mutate this gen 
    virtual void mutate(const crsGA::UserData *) override
    {
        bit = 1 - bit;
    }
    // Random creation, used for initialization
    virtual void random(const crsGA::UserData *) override
    {
        if(computeProbability() >= 50.0)
            bit = 1 - bit;
    }
    // Just some friendly way of serializing the information
    friend std::ostream &operator<<(std::ostream &os, const Gen &gen)
    {
        os << static_cast<int>(gen.bit);
        return os;
    }
};

/**
 * @brief Defines how to compute the fitness of a chromosome
 * 
 * We return -sum(bitstring), since the algorithm tries to 
 * minimize the fittnes value
 */
class ComputeFitness : public crsGA::ComputeFitnessPolicy<Gen>
{
  public:
    virtual float computeFitness(const std::vector<Gen> &genes,
                                 const crsGA::UserData *) const override
    {
        float fitness = 0.0f;
        for (const auto &g : genes)
        {
            fitness += g.bit;
        }
        return -fitness;
    }
};

/**
 * @brief Defines the initialization policy for the Population
 * 
 * @tparam ChromosomeT Type of Chromosome
 */
template <typename ChromosomeT>
class PopulationInitializationPolicy
{
  public:
    void initialize(std::vector<ChromosomeT> &chromosomes, const crsGA::UserData *data) const
    {
        for (auto &c : chromosomes)
        {
            for(auto &g: c.getGenes())
            {
                g.random(data);
            }
        }
    }
};

// Chromosome definition, internally a std::vector<Gen>
using Chromosome = crsGA::Chromosome<Gen, ComputeFitness>;
// Population definition
using Population = crsGA::Population<Chromosome, PopulationInitializationPolicy<Chromosome>>;
// Genetic algorithm implementation
using OneMaxGA = crsGA::GeneticAlgorithm<Gen, Chromosome, Population>;

int main(int, char **)
{
    // Number of bits of the onemax bitstring
    auto numGenes = 20u;
    auto populationSize = 100u;
    auto fitnessGoal = -static_cast<float>(numGenes);
    // mutation rate
    float mutationFactor = 0.5f;
    OneMaxGA ga(populationSize, numGenes, fitnessGoal);
    ga.setMutationFactor(mutationFactor);
    ga.reset();
    // run up to 100 seconds
    ga.run(10.0);
    return 0;
}
