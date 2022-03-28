#ifndef LABELED_HE_CONTROL_H
#define LABELED_HE_CONTROL_H

#include "LabeledHE/labeled_he.h"


class Labeled_he_p_control
{
	private:
		he_ct c_kp;
		gmp_randstate_t state;
		mpz_t y;
		mpz_t N;
		uint32_t msgsize;
	public:
		// Default constructor
		Labeled_he_p_control(void);
	
		// Parametrized constructor
		Labeled_he_p_control(
			const he_ct *c_kp_set,
			const mpz_t y_set,
			const mpz_t N_set,
			const uint32_t msgsize_set);
		void set_c_kp(const he_ct *c_kp_set);
		void set_y(const mpz_t y_set);
		void set_N(const mpz_t N_set);
		void set_msgsize(const uint32_t msgsize_set);
		void iterate(mpz_t c_u, const he_ct *c_e);
};

class Labeled_he_pi_control
{
	private:
		he_ct c_kp;
		he_ct c_ki;
		gmp_randstate_t state;
		mpz_t integral;
		mpz_t y;
		mpz_t N;
		uint32_t msgsize;
		uint8_t is_valid;
	public:
		// Default constructor
		Labeled_he_pi_control(void);

		// Parametrized constructor
		Labeled_he_pi_control(
			const he_ct *c_kp_set,
			const he_ct *c_ki_set,
			const mpz_t y_set,
			const mpz_t N_set,
			const uint32_t msgsize_set);
		void set_c_kp(const he_ct *c_kp_set);
		void set_c_ki(const he_ct *c_ki_set);
		void set_y(const mpz_t y_set);
		void set_N(const mpz_t N_set);
		void set_msgsize(const uint32_t msgsize_set);
		void iterate(
			mpz_t c_u,
			const he_ct *c_t,
			const he_ct *c_e,
			const mpz_t betait,
			const mpz_t betaie,
			const mpz_t betate);
};

class Labeled_he_pid_control
{
	private:
		he_ct c_kp;
		he_ct c_ki;
		he_ct c_kd;
		he_ct prev_error;
		gmp_randstate_t state;
		mpz_t integral;
		mpz_t y;
		mpz_t N;
		uint32_t msgsize;
		uint8_t is_valid;
	public:
		// Default constructor
		Labeled_he_pid_control(void);

		// Parametrized constructor
		Labeled_he_pid_control(
			const he_ct *c_kp_set,
			const he_ct *c_ki_set,
			const he_ct *c_kd_set,
			const mpz_t y_set,
			const mpz_t N_set,
			const uint32_t msgsize_set);
		void set_c_kp(const he_ct *c_kp_set);
		void set_c_ki(const he_ct *c_ki_set);
		void set_c_kd(const he_ct *c_kd_set);
		void set_y(const mpz_t y_set);
		void set_N(const mpz_t N_set);
		void set_msgsize(const uint32_t msgsize_set);
		void iterate(
			mpz_t c_u,
			const he_ct *c_t,
			const he_ct *c_t_inv,
			const he_ct *c_e,
			const mpz_t betait,
			const mpz_t betaie,
			const mpz_t betate,
			const mpz_t betadt_inv,
			const mpz_t betade,
			const mpz_t betat_inve);
};

#endif
