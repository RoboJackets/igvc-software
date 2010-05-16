
#include "OSMC_4wd_driver.hpp"

OSMC_4wd_driver::OSMC_4wd_driver(const byte FORosmc, const byte FORcoder, const byte AFTosmc, const byte AFTcoder) : FOR(FORosmc, FORcoder) , AFT(AFTosmc, AFTcoder)
{
	m_connected = true;
}
