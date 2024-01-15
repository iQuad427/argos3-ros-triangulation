#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/simulator/entities/cylinder_entity.h>
#include <argos3/plugins/simulator/entities/light_entity.h>
#include <argos3/plugins/simulator/entities/led_entity.h>
#include <argos3/plugins/simulator/media/led_medium.h>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/arena/simulator/arena_entity.h>

#include <fstream>
#include <algorithm>
#include <cstring>
#include <cerrno>

using namespace argos;

static const UInt8  NUM_ROBOTS             = 20;

class CLoopTemplateController : public CLoopFunctions {

public:

   /**
    * Class constructor
    */
   CLoopTemplateController();

   /**
    * Class destructor
    */
   virtual ~CLoopTemplateController();

   /**
    * Initializes the experiment.
    * It is executed once at the beginning of the experiment, i.e., when ARGoS is launched.
    * @param t_tree The parsed XML tree corresponding to the <loop_functions> section.
    */
   virtual void Init(TConfigurationNode& t_tree);

   void InitializeArena();

   /**
    * Resets the experiment to the state it was right after Init() was called.
    * It is executed every time you press the 'reset' button in the GUI.
    */
   virtual void Reset();

   /**
    * Undoes whatever Init() did.
    * It is executed once when ARGoS has finished the experiment.
    */
   virtual void Destroy();

   /**
    * Performs actions right before a simulation step is executed.
    */
   virtual void PreStep();

   /**
    * Performs actions right after a simulation step is executed.
    */
   virtual void PostStep();

   virtual void PostExperiment();

   /**
    * Returns the color of the floor at the specified point on.
    * @param c_position_on_plane The position at which you want to get the color.
    * @see CColor
    */
   virtual CColor GetFloorColor(const CVector2& c_position_on_plane);

   bool IsOnSource(CVector2& c_position_robot);

private:

   /**
    * The path of the output file.
    */
   std::string m_strOutFile;

   /**
    * The stream associated to the output file.
    */
   std::ofstream m_cOutFile;

   CVector2 m_cCoordSource;

   /**
    * Random number generator
    */
   CRandom::CRNG* m_pcRNG;

   void AddLights();

   /**
    * Method used to create and distribute the Arena.
    */
   void PositionArena();

   /**
    * Method used to remove the arena from the arena.
    */
   void RemoveArena();

   /**
    * The arena used in the experiment.
    */
   CArenaEntity* m_pcArena;
};
