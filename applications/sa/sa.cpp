#include <crsGA/Logger.hpp>
#include <crsGA/Common.hpp>
#include <crsGA/SimulatedAnnealing.hpp>

#include <osg/Vec3d>
#include <osg/Array>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/PrimitiveSet>
#include <osg/Point>
#include <osg/NodeVisitor>
#include <osg/LineWidth>
#include <osg/ArgumentParser>
#include <osg/OperationThread>

#include <osgGA/TrackballManipulator>

#include <osgViewer/Viewer>

#include <osgText/Text>

#include <vector>
#include <random>
#include <memory>

class PointCloud : public crsGA::UserData
{
  public:
    osg::ref_ptr<osg::Vec3dArray> points;
    osg::ref_ptr<osg::Vec4Array> colors;
    osg::BoundingBoxd boundingBox;
    osg::Vec3d center;
    double radius;
    double mutationFactor = 10;
    void init()
    {
        for (const auto &p : *points)
        {
            boundingBox.expandBy(p);
        }
        center = boundingBox.center();
        radius = boundingBox.radius() * 0.8;
    }
};

const PointCloud *asPointCloud(const crsGA::UserData *data) { return static_cast<const PointCloud *>(data); }

class Gen : public crsGA::IGen
{
  public:
    osg::Vec3d point;

  public:
    Gen()
        : point()
    {
    }
    virtual ~Gen() {}
    virtual void mutate(const crsGA::UserData *data) override
    {

        std::random_device rd;
        std::mt19937 rng(rd());
        std::uniform_real_distribution<double> random_length_uni(0.0, asPointCloud(data)->radius / asPointCloud(data)->mutationFactor);
        // move in an random direction of random length
        auto random_length = random_length_uni(rng);
        std::uniform_real_distribution<double> random_dir(-random_length, random_length);
        point += osg::Vec3d(random_dir(rng), random_dir(rng), random_dir(rng));
    }
    virtual void random(const crsGA::UserData *data) override
    {
        std::random_device rd;
        std::mt19937 rng(rd());
        std::uniform_real_distribution<double>
            random_pos_uni(-asPointCloud(data)->radius, asPointCloud(data)->radius);
        point = asPointCloud(data)->center + osg::Vec3d(random_pos_uni(rng), random_pos_uni(rng), random_pos_uni(rng));
    }
    friend std::ostream &operator<<(std::ostream &os, const Gen &gen)
    {
        os << "{" << gen.point.x() << ", " << gen.point.y() << ", " << gen.point.z() << "}";
        return os;
    }
};

template <typename T>
T smootherstep(T edge0, T edge1, T x)
{
    // Scale, and clamp x to 0..1 range
    x = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
    // Evaluate polynomial
    return x * x * x * (x * (x * 6 - 15) + 10);
}
template <typename T>
T clamp(T x, T lowerlimit, T upperlimit)
{
    if (x < lowerlimit)
        x = lowerlimit;
    if (x > upperlimit)
        x = upperlimit;
    return x;
}

class ComputeFitness : public crsGA::ComputeFitnessPolicy<Gen>
{
  public:
    virtual float computeFitness(const std::vector<Gen> &genes, const crsGA::UserData *data) const
    {
        float fitness = 0.0f;
        const PointCloud *pointcloudData = asPointCloud(data);
        for (const auto &p : *pointcloudData->points)
        {
            if (p.z() < pointcloudData->center.z())
                continue;
            double d = FLT_MAX;
            double dot_product_fitness = 0.0;
            for (size_t i = 0; i < genes.size(); i++)
            {
                double dp = (p - genes[i].point).length();
                if (dp < d)
                {
                    d = dp;
                }
                if (i > 1)
                {
                    auto v1 = genes[i].point - genes[i - 1].point;
                    v1.normalize();
                    auto v2 = genes[i - 1].point - genes[i - 2].point;
                    v2.normalize();
                    auto dot_product = (v1 * v2);
                    dot_product_fitness += 1 - dot_product;
                }
            }
            fitness += d + 0.8 * dot_product_fitness;
        }
        return fitness / genes.size();
    }
};

template <typename ChromosomeT>
class PopulationInitializationPolicy
{
  public:
    void initialize(std::vector<ChromosomeT> &chromosomes, const crsGA::UserData *data) const
    {
        for (auto &c : chromosomes)
        {
            c.getGen(0).random(data);
            //c.getGen(0).point = (*points)[0];
            auto lastIndex = c.getNumGenes() - 1;
            c.getGen(lastIndex).random(data);
            //c.getGen(lastIndex).point = (*points)[points->size() - 1];
            auto direction = c.getGen(lastIndex).point - c.getGen(0).point;
            auto direction_length = direction.length() / c.getNumGenes();
            direction.normalize();
            direction *= direction_length;
            LOG_DEBUG(c);
            for (size_t i = 1; i < c.getNumGenes() - 1; i++)
            {
                auto &gen = c.getGen(i);
                gen.point = c.getGen(i - 1).point + direction;
            }
            /*for (size_t i = 1; i < c.getNumGenes() - 1; i++)
            {
                auto &gen = c.getGen(i);
                gen.mutate(data);
            }*/
        }
    }
};

template <typename ChromosomeT,
          typename PopulationT>
class SASelectionPolicy
{
  public:
    std::vector<ChromosomeT> select(const PopulationT &population) const
    {
        std::vector<ChromosomeT> fittestList;
        std::random_device rd;
        std::mt19937 rng(rd());
        std::uniform_int_distribution<uint32_t> rIndex(0, population.getChromosomes().size() - 1);
        fittestList.push_back(population.getChromosome(rIndex(rng)));
        return fittestList;
    }
};

typedef crsGA::Chromosome<Gen, ComputeFitness> Chromosome;
typedef crsGA::Population<Chromosome, PopulationInitializationPolicy<Chromosome>> Population;
typedef crsGA::SimulatedAnnealing<Gen, Chromosome, Population> SA; //, SASelectionPolicy<Chromosome, Population>> SA;

std::shared_ptr<PointCloud> initTestLIDARPoints()
{
    auto pc = std::make_shared<PointCloud>();
    pc->points = new osg::Vec3dArray();
    pc->colors = new osg::Vec4Array();
    for (double v = -10.0; v < 10.0; v += 2.0)
    {
        pc->points->push_back(osg::Vec3d(v, v, v));
        pc->colors->push_back(osg::Vec4(1.0, 1.0, 1.0, 1.0));
    }
    pc->init();
    return pc;
}
std::shared_ptr<PointCloud> readTXTPointCloud(const std::string &filename)
{
    auto pc = std::make_shared<PointCloud>();
    pc->points = new osg::Vec3dArray();
    pc->colors = new osg::Vec4Array();
    osg::Vec3d center;
    std::ifstream file(filename);
    double x;
    while (file >> x)
    {
        osg::Vec3d p;
        osg::Vec4 c;
        p.x() = x;
        file >> p.y();
        file >> p.z();
        file >> c.r();
        file >> c.g();
        file >> c.b();
        if (pc->points->empty())
            center = p;
        pc->points->push_back(p - center);
        pc->colors->push_back(c / 255);
    }
    pc->init();
    return pc;
}

osg::ref_ptr<osg::Node> generateLiDARScene(const PointCloud &pc)
{
    osg::Geometry *geo = new osg::Geometry();
    geo->setVertexArray(pc.points.get());
    geo->setColorArray(pc.colors.get(), osg::Array::BIND_PER_VERTEX);
    geo->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, pc.points->size()));
    geo->getOrCreateStateSet()->setAttributeAndModes(new osg::Point(5.0), osg::StateAttribute::ON);
    geo->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    return geo;
}

void updateGeometryWithChromosome(osg::Geometry *geo, const Chromosome &c)
{
    auto vertices = dynamic_cast<osg::Vec3dArray *>(geo->getVertexArray());
    if (vertices)
    {
        for (size_t i = 0; i < c.getNumGenes(); i++)
        {
            vertices->at(i) = c.getGen(i).point;
        }
        vertices->dirty();
    }
}

struct UpdateChromosomeNode : public osg::NodeCallback
{
    SA *_SA;
    int _index;
    UpdateChromosomeNode(SA *SA) : _SA(SA), _index(-1) {}
    UpdateChromosomeNode(SA *SA, size_t index) : _SA(SA), _index(index) {}
    virtual void operator()(osg::Node *node, osg::NodeVisitor *)
    {
        auto geo = node->asGeometry();
        if (geo)
        {
            if (_index == -1)
            {
                const auto &c = _SA->getFittestChromosome();
                updateGeometryWithChromosome(geo, c);
                return;
            }
            const auto &c = _SA->getPopulation().getChromosome(_index);
            updateGeometryWithChromosome(geo, c);
        }
    }
};

osg::ref_ptr<osg::Geometry> generateChromosomeGeometry(const unsigned int numGenes, const osg::Vec4 &color, float size)
{
    osg::Geometry *geo = new osg::Geometry();
    geo->setDataVariance(osg::Object::DYNAMIC);
    auto vertices = new osg::Vec3dArray(numGenes);
    geo->setVertexArray(vertices);
    geo->setUseDisplayList(false);
    osg::Vec4Array *colors = new osg::Vec4Array(vertices->size());
    for (size_t i = 0; i < vertices->size(); i++)
        (*colors)[i] = color;
    geo->setColorArray(colors, osg::Array::BIND_PER_VERTEX);
    geo->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, vertices->size()));
    geo->getOrCreateStateSet()->setAttributeAndModes(new osg::LineWidth(size), osg::StateAttribute::ON);
    geo->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    return geo;
}

osg::ref_ptr<osg::Node> generatePopulationScene(SA *SA)
{
    osg::Group *g = new osg::Group();
    for (size_t i = 0; i < SA->getPopulation().getChromosomes().size(); i++)
    {
        auto cn = generateChromosomeGeometry(SA->getPopulation().getChromosome(0).getGenes().size(), osg::Vec4(0.0, 0.0, 0.0, 1.0), 1.0);
        updateGeometryWithChromosome(cn.get(), SA->getPopulation().getChromosome(i));
        cn->setUpdateCallback(new UpdateChromosomeNode(SA, i));
        g->addChild(cn.get());
    }
    return g;
}

osg::ref_ptr<osg::Node> generateFittestChromosomeScene(SA *SA, const osg::Vec4 &color = osg::Vec4(1.0, 0.0, 0.0, 1.0))
{
    auto cn = generateChromosomeGeometry(SA->getPopulation().getChromosome(0).getGenes().size(), color, 4.0);
    const auto &c = SA->getFittestChromosome();
    updateGeometryWithChromosome(cn.get(), c);
    cn->setUpdateCallback(new UpdateChromosomeNode(SA));
    return cn;
}

osg::ref_ptr<osgText::Text> generateFitnessTextScene(const osg::Vec4 &color = osg::Vec4(1.0, 0.0, 0.0, 1.0))
{
    osgText::Text *text = new osgText::Text;
    text->setDataVariance(osg::Object::DYNAMIC);
    text->setPosition(osg::Vec3f(10, 10, 0));
    text->setColor(color);
    text->setFont("times.ttf");
    text->setText("Fitness = ");
    text->setCharacterSize(20);
    text->setAxisAlignment(osgText::Text::SCREEN);

    return text;
}

osg::ref_ptr<osg::Node> generateHUDTextScene(osgText::Text *text)
{
    osg::Camera *camera = new osg::Camera;
    camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    camera->setProjectionMatrixAsOrtho2D(0, 1280, 0, 1024);
    camera->setViewMatrix(osg::Matrix::identity());
    camera->setClearMask(GL_DEPTH_BUFFER_BIT);
    camera->addChild(text);
    camera->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    return camera;
}

class UpdateOperation : public osg::Operation
{
  protected:
    SA *_SA;
    osg::ref_ptr<osgText::Text> _fitnessText;
    double _maxDuration;
    std::chrono::high_resolution_clock::time_point _t1;

  public:
    UpdateOperation(SA *SA, osgText::Text *fitnessText, double maxDuration)
        : osg::Referenced(true), Operation("UpdateOperation", true),
          _SA(SA),
          _fitnessText(fitnessText),
          _maxDuration(maxDuration),
          _t1(std::chrono::high_resolution_clock::now())
    {
    }

    virtual void operator()(osg::Object *caller)
    {
        std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - _t1);
        osgViewer::Viewer *viewer = dynamic_cast<osgViewer::Viewer *>(caller);
        if (viewer)
        {
            if (time_span.count() >= _maxDuration)
            {
                LOG_WARNING("Max duration reached");
                viewer->setDone(true);
                return;
            }
            update();
        }
        else
            _SA->step(time_span.count());
    }

    void update()
    {
        std::stringstream ss;
        ss << "Fitness = " << _SA->getFittestChromosome().getFitness();
        _fitnessText->setText(ss.str());
    }
};

int main(int argc, char **argv)
{
    osg::ArgumentParser arguments(&argc, argv);
    std::string filename;
    std::shared_ptr<PointCloud> pc;
    if (arguments.read("--txt", filename))
    {
        pc = readTXTPointCloud(filename);
    }
    else
        pc = initTestLIDARPoints();

    auto genMutationProbability = 0.5f;
    auto populationSize = 50u;
    auto numGenes = 6u;
    auto fitnessGoal = 0.01f;
    auto show = false;
    auto maxDuration = 60.0;
    auto temperature = 1000.0f;
    auto temperatureFactor = 0.01f;
    arguments.read("--genMutationProbability", genMutationProbability);
    arguments.read("--mutationFactor", pc->mutationFactor);
    arguments.read("--populationSize", populationSize);
    arguments.read("--numGenes", numGenes);
    arguments.read("--fitnessGoal", fitnessGoal);
    arguments.read("--maxDuration", maxDuration);
    arguments.read("--temperature", temperature);
    arguments.read("--temperatureFactor", temperatureFactor);
    show = arguments.read("--show");

    SA SA(populationSize, numGenes, temperature, fitnessGoal);
    SA.setMutationFactor(genMutationProbability);
    SA.setTemperatureFactor(temperatureFactor);
    SA.setUserData(pc);

    if (show)
    {
        SA.reset();
        osgViewer::Viewer viewer;
        viewer.setUpViewInWindow(100, 100, 800, 600);
        viewer.setCameraManipulator(new osgGA::TrackballManipulator());
        osg::Group *g = new osg::Group();
        g->addChild(generateLiDARScene(*pc));
        g->addChild(generatePopulationScene(&SA));
        osg::Vec4f fittestColor = osg::Vec4(1.0, 0.0, 0.0, 1.0);
        g->addChild(generateFittestChromosomeScene(&SA, fittestColor));
        auto fittestTextScene = generateFitnessTextScene(fittestColor);
        g->addChild(generateHUDTextScene(fittestTextScene.get()));

        osg::ref_ptr<osg::OperationThread> updateOperationThread = new osg::OperationThread();
        osg::ref_ptr<UpdateOperation> updateOperation = new UpdateOperation(&SA, fittestTextScene.get(), maxDuration);
        updateOperationThread->add(updateOperation.get());
        updateOperationThread->startThread();
        viewer.addUpdateOperation(updateOperation.get());

        viewer.setSceneData(g);
        viewer.realize();
        viewer.run();
    }
    else
    {
        // console app
        SA.run(maxDuration, true);
    }
    if (SA.isSolution(SA.getFittestChromosome()))
    {
        LOG_DEBUG("Solution found in generation ", SA.getGeneration());
    }
    else
    {
        LOG_DEBUG("Solution NOT found in generation ", SA.getGeneration());
    }
    LOG_DEBUG("Fitness=", SA.getFittestChromosome().getFitness(), " Chromosome: {", SA.getFittestChromosome(), "}");

    return 0;
}