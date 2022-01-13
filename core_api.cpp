/* 046267 Computer Architecture - Winter 20/21 - HW #4 */

#include "core_api.h"
#include "sim_api.h"

#include <stdio.h>

#define BLOCKED 0
#define FINE 1
#define STOPPED -1
#define IDLE -2
#define ERROR -1


/**
 * @brief The number of threads in the Simulator
 * 
 */
int threads_numSIM;

/**
 * @brief Class representing a Thread
 * @param reg[]  	Array of Register of the thread.
 * @param line  	Current line the Thread is at.
 * @param busy 		Counter of the number of Cycles the Thread is working(On Load/Store).
 * @param stopped 	If the Thread reached it's end(After HALT).
 */
class ThreadSIM
{
public:
	int reg[REGS_COUNT];
	int line;
	int busy;
	bool stopped;
	/**
	 * @brief Construct a new ThreadSIM object
	 * 
	 */
	ThreadSIM() : line(0),busy(0),stopped(false) 
	{
		for (int i = 0; i < REGS_COUNT; i++)
		{
			reg[i] = 0;
		}
	}
	/**
	 * @brief Destroy the Thread S I M object
	 * 
	 */
	~ThreadSIM() = default;
};


/**
 * @brief The MT simulation
 *  type == BLOCK : BlockMT simulation
 *  type == FINE : FineGrainedMT simulation
 *
 *	@param cycles 			num of cycles.
 *	@param instructions  	num of instructions.
 *	@param type   			type == BLOCK : BlockMT simulation.
 *				  			type == FINE : FineGrainedMT simulation.
 *	@param done 			done = 1 : the simulation is over.
 *							done = 0 : the simulation is over.
 *	@param threads			array of threads - ThreadSIM type.
 */
class Simulation
{
	public:
		int cycles;
		int instructions;
		int type;
		bool done;
		ThreadSIM *threads;
		/**
		 * @brief Default Construct a new Simulation object
		 * Deleted cause will never be called.
		 */
		Simulation() = delete;
		/**
		 * @brief Construct a new Simulation object
		 * 
		 * @param type 	type == BLOCK : BlockMT simulation.
		 *				type == FINE : FineGrainedMT simulation.
		 */
		Simulation(int type);
		/**
		 * @brief Destroy the Simulation object
		 * 
		 */
		~Simulation();
		/**
		 * @brief Get the Next Thread to Execute
		 * 
		 * @param current_thread The current Thread that was executed.
		 * @param start If it is the first time the sim is running.
		 * @return int Next Thread to Execute
		 */
		int getNextThread(int current_thread, bool start);
};

Simulation::Simulation(int type): cycles(0), instructions(0), type(type), done(false)
{
	threads = new ThreadSIM[SIM_GetThreadsNum()];
}

Simulation::~Simulation()
{
	delete threads;
}

int Simulation::getNextThread(int current_thread,bool start)
{
	done = true;
	for (int i = 0; i < threads_numSIM; i++)
	{
		if (!threads[i].stopped)
		{
			done = false;
		}
	}
	if(done)
	{
		return STOPPED;
	}
	if(type == BLOCKED) // BLOCKED
	{
		if (threads[current_thread].busy > 0 || threads[current_thread].stopped)
		{
			for (int i = current_thread; i < threads_numSIM; i++)
			{
				if(threads[i].busy == 0 && !threads[i].stopped)
				{
					return i;
				}
			}
			for (int i = 0; i < current_thread; i++)
			{
				if(threads[i].busy == 0 && !threads[i].stopped)
				{
					return i;
				}
			}
			return IDLE;			
		}
		return current_thread;
	}
	else // FINE GRAINED
	{
		if(start)
		{
			return current_thread;
		}
		current_thread++;
		current_thread = current_thread % threads_numSIM;
		
		for (int i = current_thread; i < threads_numSIM; i++)
		{
			if(threads[i].busy == 0 && !threads[i].stopped)
			{
				return i;
			}
		}
		for (int i = 0; i < current_thread; i++)
		{
			if(threads[i].busy == 0 && !threads[i].stopped)
			{
				return i;
			}
		}
		return IDLE;			
	}


	return ERROR;
}

Simulation* blockedSIM;
Simulation* fine_grainedSIM;

/**
 * @brief Executes the current instruction while updating the latancy(busy counter).
 * 
 * @param inst the current instruction
 * @param current_thread the current thread
 * @param type the MT type
 */
void updateThread(Instruction* inst, int current_thread, int type)
{
	Simulation* curr_sim;
	if(type == BLOCKED)
	{
		curr_sim = blockedSIM;
	}
	else
	{
		curr_sim = fine_grainedSIM;
	}
	switch (inst->opcode)
	{
		case CMD_LOAD:
		{
			if (inst->isSrc2Imm)
			{
				SIM_MemDataRead(curr_sim->threads[current_thread].reg[inst->src1_index] + inst->src2_index_imm,
								&curr_sim->threads[current_thread].reg[inst->dst_index]);
			}
			else
			{
				SIM_MemDataRead(curr_sim->threads[current_thread].reg[inst->src1_index] + curr_sim->threads[current_thread].reg[inst->src2_index_imm],
								&curr_sim->threads[current_thread].reg[inst->dst_index]);
			}
			
			curr_sim->threads[current_thread].busy += SIM_GetLoadLat() + 1;
			break;
		}
		case CMD_STORE:
		{
			uint32_t adrr;
			uint32_t value = curr_sim->threads[current_thread].reg[inst->src1_index];
			if(inst->isSrc2Imm)
			{
				adrr = curr_sim->threads[current_thread].reg[inst->dst_index] + inst->src2_index_imm;
			}
			else
			{
				adrr = curr_sim->threads[current_thread].reg[inst->dst_index] + curr_sim->threads[current_thread].reg[inst->src2_index_imm];
			}
			SIM_MemDataWrite(adrr,value);
			curr_sim->threads[current_thread].busy += SIM_GetStoreLat() + 1;
			break;
		}
		case CMD_HALT:
		{
			curr_sim->threads[current_thread].stopped = true;
			break;
		}
		case CMD_SUB:
		{
			curr_sim->threads[current_thread].reg[inst->dst_index] =
			curr_sim->threads[current_thread].reg[inst->src1_index] - curr_sim->threads[current_thread].reg[inst->src2_index_imm];
			break;
		}

		case CMD_SUBI:
		{
			curr_sim->threads[current_thread].reg[inst->dst_index] = 
				curr_sim->threads[current_thread].reg[inst->src1_index] - inst->src2_index_imm;
			break;
		}
		case CMD_ADD:
		{
			curr_sim->threads[current_thread].reg[inst->dst_index] =
				curr_sim->threads[current_thread].reg[inst->src1_index] + curr_sim->threads[current_thread].reg[inst->src2_index_imm];
			break;
		}
		case CMD_ADDI:
		{
			curr_sim->threads[current_thread].reg[inst->dst_index] = 
				curr_sim->threads[current_thread].reg[inst->src1_index] + inst->src2_index_imm;
			break;
		}
		case CMD_NOP:
		{
			break;
		}
		default:
			break;
		
	}
}

/**
 * @brief Update the busy counter of the Working threads.
 * 
 * @param type The type of simulator.
 */
void updateBusy(int type)
{
	Simulation* curr_sim;
	if(type == BLOCKED)
	{
		curr_sim = blockedSIM;
	}
	else
	{
		curr_sim = fine_grainedSIM;
	}
	for (int i = 0; i < threads_numSIM; i++)
	{
		if(curr_sim->threads[i].busy > 0)
		{
			curr_sim->threads[i].busy--;
		}
	}
}

void CORE_BlockedMT() 
{
	blockedSIM = new Simulation(BLOCKED);
	threads_numSIM = SIM_GetThreadsNum();
	int current_thread = 0;
	int status;
	Instruction* inst = (Instruction*)malloc(sizeof(*inst));
	while (!blockedSIM->done)
	{
		status = blockedSIM->getNextThread(current_thread,false);
		if(status == STOPPED)
		{
			break;
		}
		else if(status == IDLE)
		{
			updateBusy(BLOCKED);
			blockedSIM->cycles++;	
			continue;
		}
		else if(current_thread != status)
		{
			for (int i = 0; i < SIM_GetSwitchCycles(); i++)
			{
				updateBusy(BLOCKED);
			}
			blockedSIM->cycles += SIM_GetSwitchCycles();
		}
		current_thread = status;
		SIM_MemInstRead(blockedSIM->threads[current_thread].line,inst,current_thread);
		blockedSIM->threads[current_thread].line += 1;
		updateThread(inst,current_thread,BLOCKED);
		blockedSIM->instructions++;
		blockedSIM->cycles++;
		updateBusy(BLOCKED);
	}
	free(inst);
}

void CORE_FinegrainedMT() 
{
	bool start = true;
	fine_grainedSIM = new Simulation(FINE);
	threads_numSIM = SIM_GetThreadsNum();
	int current_thread = 0;
	int status;
	Instruction* inst = (Instruction*)malloc(sizeof(*inst));
	while (!fine_grainedSIM->done)
	{
		status = fine_grainedSIM->getNextThread(current_thread,start);
		start = false;
		if(status == STOPPED)
		{
			break;
		}
		else if(status == IDLE)
		{
			updateBusy(FINE);
			fine_grainedSIM->cycles++;	
			continue;
		}
		current_thread = status;
		SIM_MemInstRead(fine_grainedSIM->threads[current_thread].line,inst,current_thread);
		fine_grainedSIM->threads[current_thread].line += 1;
		updateThread(inst,current_thread,FINE);
		fine_grainedSIM->instructions++;
		fine_grainedSIM->cycles++;
		updateBusy(FINE);
	}
	free(inst);
}

double CORE_BlockedMT_CPI()
{
	if (blockedSIM->cycles == 0)
	{
		return 0;
	}
	double result = (double)blockedSIM->cycles / (double)(blockedSIM->instructions);
	delete blockedSIM;
	return result;
}

double CORE_FinegrainedMT_CPI()
{
	if (blockedSIM->cycles == 0)
	{
		return 0;
	}
	double result = (double)fine_grainedSIM->cycles / (double)(fine_grainedSIM->instructions);
	delete fine_grainedSIM;
	return result;
}

void CORE_BlockedMT_CTX(tcontext* context, int threadid) 
{
	
	for (int i = 0; i < REGS_COUNT; i++)
	{
		context[threadid].reg[i] = blockedSIM->threads[threadid].reg[i];
	}
	return;
}

void CORE_FinegrainedMT_CTX(tcontext* context, int threadid) 
{
	for (int i = 0; i < REGS_COUNT; i++)
	{
		context[threadid].reg[i] = fine_grainedSIM->threads[threadid].reg[i];
	}
	return;
}