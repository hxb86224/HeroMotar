#include "HeroMotarControl.h"

CHeroMotarControlManager* CHeroMotarControlManager::GetInstance()
{
    return CHeroMotarControl::instance();
}
