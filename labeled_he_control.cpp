#include "labeled_he_control.h"
#include <iostream>

// *** Proportional control *** //

// Default constructor
Labeled_he_p_control::Labeled_he_p_control(void)
{
        mpz_init(c_kp.a);
        mpz_init(c_kp.beta);
        mpz_init(y);
        mpz_init(N);
        gmp_randinit_mt(state);
        msgsize = 0;
}

// Parametrized constructor
Labeled_he_p_control::Labeled_he_p_control(
                        const he_ct *c_kp_set,
                        const mpz_t y_set,
                        const mpz_t N_set,
                        const uint32_t msgsize_set)
{
         mpz_init_set(c_kp.a, c_kp_set->a);
         mpz_init_set(c_kp.beta, c_kp_set->beta);
         mpz_init_set(y, y_set);
         mpz_init_set(N, N_set);
         gmp_randinit_mt(state);
         msgsize = msgsize_set;
}

// Change controller parameters
void Labeled_he_p_control::set_c_kp(const he_ct *c_kp_set)
{
	mpz_set(c_kp.a, c_kp_set->a);
	mpz_set(c_kp.beta, c_kp_set->beta);
}

void Labeled_he_p_control::set_y(const mpz_t y_set)
{
	mpz_set(y, y_set);
}

void Labeled_he_p_control::set_N(const mpz_t N_set)
{
	mpz_set(N, N_set);
}

void Labeled_he_p_control::set_msgsize(const uint32_t msgsize_set)
{
	msgsize = msgsize_set;
}

// Iterate controller
void Labeled_he_p_control::iterate(
			mpz_t c_u,
		       	const he_ct *c_e)
{
	he_eval_mult(c_u, state, c_e, &c_kp, y, N, msgsize);
}

// *** Proportional-Integral control *** //

// Default constructor
Labeled_he_pi_control::Labeled_he_pi_control(void)
{
	mpz_init(c_kp.a);
	mpz_init(c_kp.beta);
	mpz_init(c_ki.a);
	mpz_init(c_ki.beta);
	gmp_randinit_mt(state);
	mpz_init_set_ui(integral, 0);
	mpz_init(y);
	mpz_init(N);
	is_valid = 0;
}

// Parametrized constructor
Labeled_he_pi_control::Labeled_he_pi_control(
		const he_ct *c_kp_set,
		const he_ct *c_ki_set,
		const mpz_t y_set,
		const mpz_t N_set,
		uint32_t msgsize_set)
{
	mpz_init_set(c_kp.a, c_kp_set->a);
	mpz_init_set(c_kp.beta, c_kp_set->beta);
	mpz_init_set(c_ki.a, c_ki_set->a);
	mpz_init_set(c_ki.beta, c_ki_set->beta);
	gmp_randinit_mt(state);
	mpz_init_set_ui(integral, 0);
	mpz_init_set(y, y_set);
	mpz_init_set(N, N_set);
	msgsize = msgsize_set;
	is_valid = 0;
}

void Labeled_he_pi_control::set_c_kp(const he_ct *c_kp_set)
{
	mpz_set(c_kp.a, c_kp_set->a);
	mpz_set(c_kp.beta, c_kp_set->beta);
}

void Labeled_he_pi_control::set_c_ki(const he_ct *c_ki_set)
{
	mpz_set(c_ki.a, c_ki_set->a);
	mpz_set(c_ki.beta, c_ki_set->beta);
}

void Labeled_he_pi_control::set_y(const mpz_t y_set)
{
	mpz_set(y, y_set);
}

void Labeled_he_pi_control::set_N(const mpz_t N_set)
{
	mpz_set(N, N_set);
}

void Labeled_he_pi_control::set_msgsize(const uint32_t msgsize_set)
{
	msgsize = msgsize_set;
}

void Labeled_he_pi_control::iterate(
		mpz_t c_u,
		const he_ct *c_t,
		const he_ct *c_e,
		const mpz_t betait,
		const mpz_t betaie,
		const mpz_t betate)
{
	mpz_t c_p, c_i;
	mpz_init(c_p);
	mpz_init(c_i);

	// Proportional part
	he_eval_mult(c_p, state, &c_kp, c_e, y, N, msgsize);

	// Update integral part
	he_eval_mult_3(c_i, state, &c_ki, c_e, c_t, betaie, betait, betate, y, N, msgsize);
	if (!is_valid)
	{
		mpz_set(integral, c_i);
		is_valid = 1;
	}
	else 
	{
		he_eval_add(integral, integral, c_i, N);
	}

	// Return sum of proportional and integral part
	he_eval_add(c_u, c_p, integral, N);
}

// *** Proportional-Integral-Derivative (PID) control *** //

// Default constructor
Labeled_he_pid_control::Labeled_he_pid_control(void)
{
	mpz_init(c_kp.a);
	mpz_init(c_kp.beta);
	mpz_init(c_ki.a);
	mpz_init(c_ki.beta);
	mpz_init(c_kd.a);
	mpz_init(c_kd.beta);
	mpz_init(prev_error.a);
	mpz_init(prev_error.beta);
	gmp_randinit_mt(state);
	mpz_init_set_ui(integral, 0);
	mpz_init(y);
	mpz_init(N);
	is_valid = 0;
}

// Parametrized constructor
Labeled_he_pid_control::Labeled_he_pid_control(
		const he_ct *c_kp_set,
		const he_ct *c_ki_set,
		const he_ct *c_kd_set,
		const mpz_t y_set,
		const mpz_t N_set,
		uint32_t msgsize_set)
{
	mpz_init_set(c_kp.a, c_kp_set->a);
	mpz_init_set(c_kp.beta, c_kp_set->beta);
	mpz_init_set(c_ki.a, c_ki_set->a);
	mpz_init_set(c_ki.beta, c_ki_set->beta);
	mpz_init_set(c_kd.a, c_kd_set->a);
	mpz_init_set(c_kd.beta, c_kd_set->beta);
	mpz_init(prev_error.a);
	mpz_init(prev_error.beta);
	gmp_randinit_mt(state);
	mpz_init_set_ui(integral, 0);
	mpz_init_set(y, y_set);
	mpz_init_set(N, N_set);
	msgsize = msgsize_set;
	is_valid = 0;
}

void Labeled_he_pid_control::set_c_kp(const he_ct *c_kp_set)
{
	mpz_set(c_kp.a, c_kp_set->a);
	mpz_set(c_kp.beta, c_kp_set->beta);
}

void Labeled_he_pid_control::set_c_ki(const he_ct *c_ki_set)
{
	mpz_set(c_ki.a, c_ki_set->a);
	mpz_set(c_ki.beta, c_ki_set->beta);
}

void Labeled_he_pid_control::set_c_kd(const he_ct *c_kd_set)
{
	mpz_set(c_kd.a, c_kd_set->a);
	mpz_set(c_kd.beta, c_kd_set->beta);
}

void Labeled_he_pid_control::set_y(const mpz_t y_set)
{
	mpz_set(y, y_set);
}

void Labeled_he_pid_control::set_N(const mpz_t N_set)
{
	mpz_set(N, N_set);
}

void Labeled_he_pid_control::set_msgsize(const uint32_t msgsize_set)
{
	msgsize = msgsize_set;
}

void Labeled_he_pid_control::iterate(
		mpz_t c_u,
		const he_ct *c_t,
		const he_ct *c_t_inv,
		const he_ct *c_e,
		const mpz_t betait,
		const mpz_t betaie,
		const mpz_t betate,
		const mpz_t betadt_inv,
		const mpz_t betade,
		const mpz_t betat_inve)
{
	mpz_t c_p, c_i, c_d;
	mpz_init(c_p);
	mpz_init(c_i);
	mpz_init_set_ui(c_d, 0);

	//he_eval_sub(&delta_tau, c_t, &prev_timestep, N, msgsize);
	
	uint8_t pre_is_valid = is_valid;
	
	// Proportional part
	he_eval_mult(c_p, state, &c_kp, c_e, y, N, msgsize);

	// Update integral part
	he_eval_mult_3(c_i, state, &c_ki, c_e, c_t, betaie, betait, betate, y, N, msgsize);
	if (!is_valid)
	{
		mpz_set(integral, c_i);
		is_valid = 1;
	}
	else 
	{
		he_eval_add(integral, integral, c_i, N);
	}

	// Return sum of proportional and integral part
	he_eval_add(c_u, c_p, integral, N);

	// Derivative part
	if (!pre_is_valid)
	{
		// First sample, no derivative action
		mpz_set(prev_error.a, c_e->a);
		mpz_set(prev_error.beta, c_e->beta);
	}
	else
	{
		// Change in error since last
		he_ct delta_e;
		mpz_init(delta_e.a);
		mpz_init(delta_e.beta);
		he_eval_sub(&delta_e, c_e, &prev_error, N, msgsize);
		he_eval_mult_3(c_d, state, &c_kd, &delta_e, c_t_inv, betade, betadt_inv, betat_inve, y, N, msgsize);
		// Add the derivative part
		he_eval_add(c_u, c_u, c_d, N);

		mpz_set(prev_error.a, c_e->a);
		mpz_set(prev_error.beta, c_e->beta);
	}
}
