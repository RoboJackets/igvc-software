
#include "OSMC_4wd_driver.hpp"

OSMC_4wd_driver::OSMC_4wd_driver(byte FORosmc, byte FORcoder, byte AFTosmc, byte AFTcoder) : FOR(FORosmc, FORcoder) , AFT(AFTosmc, AFTcoder)
{
	m_connected = true;
}
