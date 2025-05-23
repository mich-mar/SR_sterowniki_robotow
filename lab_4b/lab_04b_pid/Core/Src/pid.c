/*
* pid.c
*
* Created on: 09.03.2018
* Author: Wojciech Domski
*/

#include "pid.h"

/**
 * @brief Inicjalizacja regulatora PID
 *
 * Ta funkcja inicjalizuje wszystkie parametry regulatora PID i zmienne stanu.
 * Wzmocnienia zmiennoprzecinkowe są konwertowane na reprezentację stałoprzecinkową
 * używając określonej liczby bitów ułamkowych.
 *
 * Konwersja stałoprzecinkowa używa wzoru: wartość_stała = wartość_float * 2^f
 * gdzie f to liczba bitów ułamkowych.
 *
 * @param pid Wskaźnik do struktury regulatora PID do inicjalizacji
 * @param p Wzmocnienie proporcjonalne (Kp) jako wartość zmiennoprzecinkowa
 * @param i Wzmocnienie całkujące (Ki) jako wartość zmiennoprzecinkowa
 * @param d Wzmocnienie różniczkujące (Kd) jako wartość zmiennoprzecinkowa
 * @param f Liczba bitów ułamkowych dla arytmetyki stałoprzecinkowej (zazwyczaj 8-16)
 * @param dt_ms Czas próbkowania w milisekundach
 *
 * @note Wszystkie wartości ograniczeń są inicjalizowane do maksymalnych/minimalnych
 *       wartości int32_t. Powinny być odpowiednio ustawione po inicjalizacji.
 */
void pid_init(cpid_t *pid, float p, float i, float d, uint8_t f, int32_t dt_ms)
{
    uint32_t k;

    // Oblicz współczynnik skalowania (2^f) dla arytmetyki stałoprzecinkowej
    pid->power = 1;
    for (k = 0; k < f; ++k)
    {
        pid->power *= 2;
    }

    // Zapisz liczbę bitów ułamkowych
    pid->f = f;

    // Konwertuj wzmocnienia zmiennoprzecinkowe na reprezentację stałoprzecinkową
    pid->p = (int32_t)(p * pid->power);
    pid->i = (int32_t)(i * pid->power);
    pid->d = (int32_t)(d * pid->power);

    // Inicjalizuj wartości składowych na zero
    pid->p_val = 0;
    pid->i_val = 0;
    pid->d_val = 0;

    // Inicjalizuj ograniczenia składowych do maksymalnego zakresu
    // Te powinny być ustawione na odpowiednie wartości po inicjalizacji
    pid->p_max = INT32_MAX;
    pid->p_min = INT32_MIN;

    pid->i_max = INT32_MAX;
    pid->i_min = INT32_MIN;

    pid->d_max = INT32_MAX;
    pid->d_min = INT32_MIN;

    // Inicjalizuj zmienne stanu
    pid->e_last = 0; // Poprzedni błąd dla obliczeń różniczkujących
    pid->sum = 0;    // Skumulowany błąd dla obliczeń całkujących

    // Inicjalizuj ograniczenia całkowitego wyjścia do maksymalnego zakresu
    pid->total_max = INT32_MAX;
    pid->total_min = INT32_MIN;

    // Zapisz czas próbkowania w milisekundach
    pid->dt_ms = dt_ms;
}

/**
 * @brief Oblicza wyjście sterowania PID
 *
 * Ta funkcja wykonuje jedną iterację algorytmu sterowania PID.
 * Oblicza składowe proporcjonalną, całkującą i różniczkującą
 * i łączy je, aby wytworzyć wyjście sterowania.
 *
 * Implementowane równanie PID to:
 * u(t) = Kp*e(t) + Ki*∫e(t)dt + Kd*de(t)/dt
 *
 * Gdzie:
 * - e(t) = dv - mv (błąd = setpoint - zmienna_procesowa)
 * - Składowa proporcjonalna: Kp * e(t)
 * - Składowa całkująca: Ki * suma(e(t) * dt)
 * - Składowa różniczkująca: Kd * (e(t) - e(t-1)) / dt
 *
 * @param pid Wskaźnik do struktury regulatora PID
 * @param mv Wartość mierzona (zmienna procesowa/sprzężenie zwrotne)
 * @param dv Wartość zadana (setpoint/odniesienie)
 * @return Wartość wyjściowa sterowania (przeskalowana w dół ze stałoprzecinkowej)
 *
 * @note Funkcja zawiera ograniczenia nasycenia dla każdej składowej
 *       aby zapobiec przepełnieniu i implementować anti-windup dla składowej całkującej.
 */
int32_t pid_calc(cpid_t *pid, int32_t mv, int32_t dv)
{
    int32_t p, i, d, e, total;

    // Zapisz aktualne wartości mierzoną i zadaną
    pid->mv = mv;
    pid->dv = dv;

    // Oblicz błąd: e(t) = setpoint - zmienna_procesowa
    e = dv - mv;

    // SKŁADOWA PROPORCJONALNA
    // P(t) = Kp * e(t)
    p = pid->p * e;
    // Zastosuj ograniczenia nasycenia aby zapobiec przepełnieniu
    if (p > pid->p_max)
        p = pid->p_max;
    else if (p < pid->p_min)
        p = pid->p_min;
    // Przeskaluj w dół aby otrzymać rzeczywistą wartość proporcjonalną
    pid->p_val = p >> pid->f;

    // SKŁADOWA CAŁKUJĄCA
    // I(t) = Ki * ∫e(t)dt = Ki * suma(e(t) * dt)
    // dt_ms/1000 konwertuje milisekundy na sekundy dla właściwego skalowania czasu
    i = pid->sum;
    i += (pid->i * e * pid->dt_ms) / 1000; // Akumuluj błąd w czasie
    // Zastosuj ograniczenia nasycenia dla ochrony anti-windup
    if (i > pid->i_max)
        i = pid->i_max;
    else if (i < pid->i_min)
        i = pid->i_min;
    pid->sum = i; // Zapisz skumulowaną sumę dla następnej iteracji
    // Przeskaluj w dół aby otrzymać rzeczywistą wartość całkującą
    pid->i_val = i >> pid->f;

    // SKŁADOWA RÓŻNICZKUJĄCA
    // D(t) = Kd * de(t)/dt = Kd * (e(t) - e(t-1)) / dt
    // 1000/dt_ms konwertuje milisekundy na sekundy dla właściwego skalowania czasu
    d = (pid->d * (e - pid->e_last) * 1000) / pid->dt_ms;
    // Zastosuj ograniczenia nasycenia aby zapobiec uderzeniu różniczkującemu
    if (d > pid->d_max)
        d = pid->d_max;
    else if (d < pid->d_min)
        d = pid->d_min;
    // Przeskaluj w dół aby otrzymać rzeczywistą wartość różniczkującą
    pid->d_val = d >> pid->f;

    // CAŁKOWITE WYJŚCIE PID
    // Połącz wszystkie trzy składowe
    total = p + i + d;
    // Zastosuj końcowe ograniczenia nasycenia wyjścia
    if (total > pid->total_max)
        total = pid->total_max;
    else if (total < pid->total_min)
        total = pid->total_min;

    // Przeskaluj w dół końcowy wynik i zapisz jako wyjście sterowania
    pid->control = total >> pid->f;

    // Zapisz aktualny błąd dla następnych obliczeń różniczkujących
    pid->e_last = e;

    return pid->control;
}

/**
 * @brief Konwertuje wartość zmiennoprzecinkową na stałoprzecinkową
 *
 * Ta funkcja pomocnicza konwertuje wartość zmiennoprzecinkową na
 * reprezentację stałoprzecinkową używaną przez regulator PID.
 *
 * @param pid Wskaźnik do struktury regulatora PID (używany dla współczynnika skalowania)
 * @param v Wartość zmiennoprzecinkowa do konwersji
 * @return Reprezentacja stałoprzecinkowa wartości wejściowej
 *
 * @note Ta funkcja jest przydatna do ustawiania ograniczeń i innych parametrów
 *       w tym samym formacie stałoprzecinkowym używanym wewnętrznie przez regulator.
 */
int32_t pid_scale(cpid_t *pid, float v)
{
    return v * pid->power;
}