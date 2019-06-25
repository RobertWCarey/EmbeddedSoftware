/*!
 * @addtogroup DOR_module DOR module documentation
 * @{
 */
/*! @file
 *
 *  @brief Routines for the accelerometer
 *
 *  This contains the functions for operating the DOR
 *
 *  @author Robert Carey
 *  @date 2019-06-09
 */

#ifndef DOR_H
#define DOR_H

#include "analog.h"
#include "types.h"
#include "MK70F12.h"
#include "OS.h"
#include "PE_types.h"

typedef struct
{
  uint32_t moduleClk;         /*!< The module clock rate in Hz. */
  TOSThreadParams* Channel0Params;  /*!< Thread parameters for Channel0. */
  TOSThreadParams* Channel1Params;  /*!< Thread parameters for Channel0. */
  TOSThreadParams* Channel2Params;  /*!< Thread parameters for Channel0. */
  TOSThreadParams* TripParams;
} TDORSetup;

typedef struct ChannelThreadData
{
  OS_ECB* semaphore;
  uint8_t channelNb;
  float irms;
  int16_t samples[16];
  int16_t sample;
  bool timerStatus;
  uint32_t currentTimeCount;
  uint32_t tripTime;
  float offset1;
  float offset2;
  uint8_t numberOfSamples;
  uint8_t crossing;
  float frequency[3];
} TAnalogThreadData;

typedef struct
{
    uint32_t y;
    double x;
} TIDMTData;

static const uint32_t INV_TRIP_TIME[1898] =
{
   2.3675e+05,1.7841e+05,1.434e+05,1.2006e+05,1.0339e+05,90885,81158,73374,67006,61697,57205,53354,50015,47093,44515,42222,40171,38324,36652,35132,33744,32471,31300,30218,
   29217,28286,27420,26611,25853,25143,24476,23848,23255,22695,22166,21664,21187,20734,20303,19893,19501,19127,18769,18427,18100,17785,17484,17194,16916,16648,16390,16142,
   15903,15672,15449,15233,15025,14824,14629,14440,14257,14080,13908,13742,13580,13423,13270,13122,12978,12838,12701,12568,12439,12313,12190,12070,11953,11839,11728,11619,
   11513,11410,11309,11210,11113,11019,10926,10836,10748,10661,10576,10493,10412,10332,10254,10178,10103,10029,9957,9886,9817,9749,9682,9616,9552,9488,9426,9365,9305,9246,
   9188,9131,9075,9020,8966,8912,8860,8808,8758,8708,8658,8610,8562,8515,8469,8424,8379,8334,8291,8248,8206,8164,8123,8082,8042,8003,7964,7926,7888,7851,7814,7778,7742,7707,
   7672,7637,7603,7570,7537,7504,7472,7440,7408,7377,7346,7316,7286,7256,7227,7198,7169,7141,7113,7085,7058,7031,7004,6978,6952,6926,6900,6875,6850,6825,6801,6777,6753,6729,
   6705,6682,6659,6636,6614,6592,6570,6548,6526,6505,6484,6463,6442,6421,6401,6381,6361,6341,6321,6302,6283,6264,6245,6226,6207,6189,6171,6153,6135,6117,6100,6082,6065,6048,
   6031,6014,5998,5981,5965,5948,5932,5916,5901,5885,5869,5854,5838,5823,5808,5793,5778,5764,5749,5735,5720,5706,5692,5678,5664,5650,5637,5623,5610,5596,5583,5570,5557,5544,
   5531,5518,5505,5493,5480,5468,5455,5443,5431,5419,5407,5395,5383,5372,5360,5348,5337,5325,5314,5303,5292,5281,5270,5259,5248,5237,5226,5216,5205,5195,5184,5174,5163,5153,
   5143,5133,5123,5113,5103,5093,5083,5074,5064,5054,5045,5035,5026,5017,5007,4998,4989,4980,4971,4962,4953,4944,4935,4926,4917,4909,4900,4891,4883,4874,4866,4857,4849,4841,
   4833,4824,4816,4808,4800,4792,4784,4776,4768,4760,4753,4745,4737,4729,4722,4714,4707,4699,4692,4684,4677,4670,4662,4655,4648,4641,4633,4626,4619,4612,4605,4598,4591,4584,
   4578,4571,4564,4557,4550,4544,4537,4530,4524,4517,4511,4504,4498,4491,4485,4479,4472,4466,4460,4454,4447,4441,4435,4429,4423,4417,4411,4405,4399,4393,4387,4381,4375,4369,
   4364,4358,4352,4346,4341,4335,4329,4324,4318,4313,4307,4302,4296,4291,4285,4280,4274,4269,4264,4258,4253,4248,4242,4237,4232,4227,4222,4217,4211,4206,4201,4196,4191,4186,
   4181,4176,4171,4166,4161,4157,4152,4147,4142,4137,4133,4128,4123,4118,4114,4109,4104,4100,4095,4090,4086,4081,4077,4072,4068,4063,4059,4054,4050,4045,4041,4037,4032,4028,
   4024,4019,4015,4011,4006,4002,3998,3994,3989,3985,3981,3977,3973,3969,3965,3960,3956,3952,3948,3944,3940,3936,3932,3928,3924,3920,3916,3913,3909,3905,3901,3897,3893,3889,
   3886,3882,3878,3874,3870,3867,3863,3859,3856,3852,3848,3844,3841,3837,3834,3830,3826,3823,3819,3816,3812,3809,3805,3801,3798,3794,3791,3788,3784,3781,3777,3774,3770,3767,
   3764,3760,3757,3754,3750,3747,3744,3740,3737,3734,3730,3727,3724,3721,3717,3714,3711,3708,3705,3701,3698,3695,3692,3689,3686,3683,3679,3676,3673,3670,3667,3664,3661,3658,
   3655,3652,3649,3646,3643,3640,3637,3634,3631,3628,3625,3622,3619,3616,3613,3611,3608,3605,3602,3599,3596,3593,3591,3588,3585,3582,3579,3577,3574,3571,3568,3565,3563,3560,
   3557,3555,3552,3549,3546,3544,3541,3538,3536,3533,3530,3528,3525,3522,3520,3517,3515,3512,3509,3507,3504,3502,3499,3497,3494,3492,3489,3486,3484,3481,3479,3476,3474,3471,
   3469,3466,3464,3462,3459,3457,3454,3452,3449,3447,3445,3442,3440,3437,3435,3433,3430,3428,3426,3423,3421,3418,3416,3414,3412,3409,3407,3405,3402,3400,3398,3395,3393,3391,
   3389,3386,3384,3382,3380,3377,3375,3373,3371,3369,3366,3364,3362,3360,3358,3355,3353,3351,3349,3347,3345,3343,3340,3338,3336,3334,3332,3330,3328,3326,3324,3321,3319,3317,
   3315,3313,3311,3309,3307,3305,3303,3301,3299,3297,3295,3293,3291,3289,3287,3285,3283,3281,3279,3277,3275,3273,3271,3269,3267,3265,3263,3261,3259,3257,3255,3253,3252,3250,
   3248,3246,3244,3242,3240,3238,3236,3234,3233,3231,3229,3227,3225,3223,3221,3220,3218,3216,3214,3212,3210,3209,3207,3205,3203,3201,3200,3198,3196,3194,3192,3191,3189,3187,
   3185,3184,3182,3180,3178,3177,3175,3173,3171,3170,3168,3166,3165,3163,3161,3159,3158,3156,3154,3153,3151,3149,3148,3146,3144,3143,3141,3139,3138,3136,3134,3133,3131,3129,
   3128,3126,3124,3123,3121,3120,3118,3116,3115,3113,3112,3110,3108,3107,3105,3104,3102,3100,3099,3097,3096,3094,3093,3091,3089,3088,3086,3085,3083,3082,3080,3079,3077,3076,
   3074,3073,3071,3070,3068,3066,3065,3063,3062,3060,3059,3058,3056,3055,3053,3052,3050,3049,3047,3046,3044,3043,3041,3040,3038,3037,3036,3034,3033,3031,3030,3028,3027,3025,
   3024,3023,3021,3020,3018,3017,3016,3014,3013,3011,3010,3009,3007,3006,3004,3003,3002,3000,2999,2998,2996,2995,2993,2992,2991,2989,2988,2987,2985,2984,2983,2981,2980,2979,
   2977,2976,2975,2973,2972,2971,2969,2968,2967,2965,2964,2963,2961,2960,2959,2958,2956,2955,2954,2952,2951,2950,2949,2947,2946,2945,2943,2942,2941,2940,2938,2937,2936,2935,
   2933,2932,2931,2930,2928,2927,2926,2925,2923,2922,2921,2920,2918,2917,2916,2915,2914,2912,2911,2910,2909,2908,2906,2905,2904,2903,2902,2900,2899,2898,2897,2896,2894,2893,
   2892,2891,2890,2889,2887,2886,2885,2884,2883,2882,2880,2879,2878,2877,2876,2875,2873,2872,2871,2870,2869,2868,2867,2865,2864,2863,2862,2861,2860,2859,2858,2856,2855,2854,
   2853,2852,2851,2850,2849,2848,2846,2845,2844,2843,2842,2841,2840,2839,2838,2837,2836,2834,2833,2832,2831,2830,2829,2828,2827,2826,2825,2824,2823,2822,2821,2820,2818,2817,
   2816,2815,2814,2813,2812,2811,2810,2809,2808,2807,2806,2805,2804,2803,2802,2801,2800,2799,2798,2797,2796,2795,2794,2793,2792,2791,2790,2789,2788,2787,2786,2785,2784,2783,
   2782,2781,2780,2779,2778,2777,2776,2775,2774,2773,2772,2771,2770,2769,2768,2767,2766,2765,2764,2763,2762,2761,2760,2759,2758,2757,2756,2755,2754,2753,2752,2751,2750,2749,
   2749,2748,2747,2746,2745,2744,2743,2742,2741,2740,2739,2738,2737,2736,2735,2735,2734,2733,2732,2731,2730,2729,2728,2727,2726,2725,2724,2723,2723,2722,2721,2720,2719,2718,
   2717,2716,2715,2714,2714,2713,2712,2711,2710,2709,2708,2707,2706,2706,2705,2704,2703,2702,2701,2700,2699,2699,2698,2697,2696,2695,2694,2693,2692,2692,2691,2690,2689,2688,
   2687,2686,2686,2685,2684,2683,2682,2681,2681,2680,2679,2678,2677,2676,2675,2675,2674,2673,2672,2671,2670,2670,2669,2668,2667,2666,2665,2665,2664,2663,2662,2661,2661,2660,
   2659,2658,2657,2656,2656,2655,2654,2653,2652,2652,2651,2650,2649,2648,2648,2647,2646,2645,2644,2644,2643,2642,2641,2640,2640,2639,2638,2637,2636,2636,2635,2634,2633,2633,
   2632,2631,2630,2629,2629,2628,2627,2626,2626,2625,2624,2623,2622,2622,2621,2620,2619,2619,2618,2617,2616,2616,2615,2614,2613,2613,2612,2611,2610,2610,2609,2608,2607,2607,
   2606,2605,2604,2604,2603,2602,2601,2601,2600,2599,2598,2598,2597,2596,2595,2595,2594,2593,2592,2592,2591,2590,2590,2589,2588,2587,2587,2586,2585,2585,2584,2583,2582,2582,
   2581,2580,2580,2579,2578,2577,2577,2576,2575,2575,2574,2573,2572,2572,2571,2570,2570,2569,2568,2568,2567,2566,2565,2565,2564,2563,2563,2562,2561,2561,2560,2559,2558,2558,
   2557,2556,2556,2555,2554,2554,2553,2552,2552,2551,2550,2550,2549,2548,2548,2547,2546,2546,2545,2544,2544,2543,2542,2542,2541,2540,2540,2539,2538,2538,2537,2536,2536,2535,
   2534,2534,2533,2532,2532,2531,2530,2530,2529,2528,2528,2527,2526,2526,2525,2524,2524,2523,2523,2522,2521,2521,2520,2519,2519,2518,2517,2517,2516,2516,2515,2514,2514,2513,
   2512,2512,2511,2510,2510,2509,2509,2508,2507,2507,2506,2505,2505,2504,2504,2503,2502,2502,2501,2500,2500,2499,2499,2498,2497,2497,2496,2496,2495,2494,2494,2493,2492,2492,
   2491,2491,2490,2489,2489,2488,2488,2487,2486,2486,2485,2485,2484,2483,2483,2482,2482,2481,2480,2480,2479,2479,2478,2477,2477,2476,2476,2475,2474,2474,2473,2473,2472,2472,
   2471,2470,2470,2469,2469,2468,2467,2467,2466,2466,2465,2465,2464,2463,2463,2462,2462,2461,2461,2460,2459,2459,2458,2458,2457,2457,2456,2455,2455,2454,2454,2453,2453,2452,
   2451,2451,2450,2450,2449,2449,2448,2447,2447,2446,2446,2445,2445,2444,2444,2443,2442,2442,2441,2441,2440,2440,2439,2439,2438,2437,2437,2436,2436,2435,2435,2434,2434,2433,
   2433,2432,2431,2431,2430,2430,2429,2429,2428,2428,2427,2427,2426,2426,2425,2424,2424,2423,2423,2422,2422,2421,2421,2420,2420,2419,2419,2418,2418,2417,2416,2416,2415,2415,
   2414,2414,2413,2413,2412,2412,2411,2411,2410,2410,2409,2409,2408,2408,2407,2407,2406,2405,2405,2404,2404,2403,2403,2402,2402,2401,2401,2400,2400,2399,2399,2398,2398,2397,
   2397,2396,2396,2395,2395,2394,2394,2393,2393,2392,2392,2391,2391,2390,2390,2389,2389,2388,2388,2387,2387,2386,2386,2385,2385,2384,2384,2383,2383,2382,2382,2381,2381,2380,
   2380,2379,2379,2378,2378,2377,2377,2376,2376,2375,2375,2374,2374,2373,2373,2372,2372,2371,2371,2371,2370,2370,2369,2369,2368,2368,2367,2367,2366,2366,2365,2365,2364,2364,
   2363,2363,2362,2362,2361,2361,2360,2360,2360,2359,2359,2358,2358,2357,2357,2356,2356,2355,2355,2354,2354,2353,2353,2353,2352,2352,2351,2351,2350,2350,2349,2349,2348,2348,
   2347,2347,2346,2346,2346,2345,2345,2344,2344,2343,2343,2342,2342,2341,2341,2341,2340,2340,2339,2339,2338,2338,2337,2337,2336,2336,2336,2335,2335,2334,2334,2333,2333,2332,
   2332,2332,2331,2331,2330,2330,2329,2329,2328,2328,2328,2327,2327,2326,2326,2325,2325,2324,2324,2324,2323,2323,2322,2322,2321,2321,2321,2320,2320,2319,2319,2318,2318,2317,
   2317,2317,2316,2316,2315,2315,2314,2314,2314,2313,2313,2312,2312,2311,2311,2311,2310,2310,2309,2309,2308,2308,2308,2307,2307,2306,2306,2306,2305,2305,2304,2304,2303,2303,
   2303,2302,2302,2301,2301,2300,2300,2300,2299,2299,2298,2298,2298,2297,2297,2296,2296,2295,2295,2295,2294,2294,2293,2293,2293,2292,2292,2291,2291,2291,2290,2290,2289,2289,
   2288,2288,2288,2287,2287,2286,2286,2286,2285,2285,2284,2284,2284,2283,2283,2282,2282,2282,2281,2281,2280,2280,2280,2279,2279,2278,2278,2278,2277,2277,2276,2276,2276,2275,
   2275,2274,2274,2274,2273,2273,2272,2272,2272,2271,2271,2270,2270,2270,2269,2269,2269,2268,2268,2267
};

static const uint32_t VINV_TRIP_TIME[1898] =
{
    4.5e+05,3.375e+05,2.7e+05,2.25e+05,1.9286e+05,1.6875e+05,1.5e+05,1.35e+05,1.2273e+05,1.125e+05,1.0385e+05,96429,90000,84375,79412,75000,71053,67500,64286,61364,58696,
    56250,54000,51923,50000,48214,46552,45000,43548,42187,40909,39706,38571,37500,36486,35526,34615,33750,32927,32143,31395,30682,30000,29348,28723,28125,27551,27000,26471,
    25962,25472,25000,24545,24107,23684,23276,22881,22500,22131,21774,21429,21094,20769,20455,20149,19853,19565,19286,19014,18750,18493,18243,18000,17763,17532,17308,17089,
    16875,16667,16463,16265,16071,15882,15698,15517,15341,15169,15000,14835,14674,14516,14362,14211,14063,13918,13776,13636,13500,13366,13235,13107,12981,12857,12736,12617,
    12500,12385,12273,12162,12054,11947,11842,11739,11638,11538,11441,11345,11250,11157,11066,10976,10887,10800,10714,10630,10547,10465,10385,10305,10227,10150,10075,10000,
    9926,9854,9783,9712,9643,9574,9507,9441,9375,9310,9247,9184,9122,9060,9000,8940,8882,8824,8766,8710,8654,8599,8544,8491,8438,8385,8333,8282,8232,8182,8133,8084,8036,7988,
    7941,7895,7849,7803,7759,7714,7670,7627,7584,7542,7500,7459,7418,7377,7337,7297,7258,7219,7181,7143,7105,7068,7031,6995,6959,6923,6888,6853,6818,6784,6750,6716,6683,6650,
    6618,6585,6553,6522,6490,6459,6429,6398,6368,6338,6308,6279,6250,6221,6193,6164,6136,6109,6081,6054,6027,6000,5973,5947,5921,5895,5870,5844,5819,5794,5769,5745,5720,5696,
    5672,5649,5625,5602,5579,5556,5533,5510,5488,5466,5444,5422,5400,5378,5357,5336,5315,5294,5273,5253,5233,5212,5192,5172,5153,5133,5114,5094,5075,5056,5037,5019,5000,4982,
    4963,4945,4927,4909,4891,4874,4856,4839,4821,4804,4787,4770,4754,4737,4720,4704,4688,4671,4655,4639,4623,4608,4592,4576,4561,4545,4530,4515,4500,4485,4470,4455,4441,4426,
    4412,4397,4383,4369,4355,4341,4327,4313,4299,4286,4272,4259,4245,4232,4219,4206,4193,4180,4167,4154,4141,4128,4116,4103,4091,4079,4066,4054,4042,4030,4018,4006,3994,3982,
    3971,3959,3947,3936,3924,3913,3902,3890,3879,3868,3857,3846,3835,3824,3814,3803,3792,3782,3771,3760,3750,3740,3729,3719,3709,3699,3689,3678,3668,3659,3649,3639,3629,3619,
    3610,3600,3590,3581,3571,3562,3553,3543,3534,3525,3516,3506,3497,3488,3479,3470,3462,3453,3444,3435,3426,3418,3409,3401,3392,3383,3375,3367,3358,3350,3342,3333,3325,3317,
    3309,3301,3293,3285,3277,3269,3261,3253,3245,3237,3230,3222,3214,3207,3199,3191,3184,3176,3169,3162,3154,3147,3140,3132,3125,3118,3111,3103,3096,3089,3082,3075,3068,3061,
    3054,3047,3041,3034,3027,3020,3013,3007,3000,2993,2987,2980,2974,2967,2961,2954,2948,2941,2935,2928,2922,2916,2909,2903,2897,2891,2885,2878,2872,2866,2860,2854,2848,2842,
    2836,2830,2824,2818,2812,2807,2801,2795,2789,2784,2778,2772,2766,2761,2755,2749,2744,2738,2733,2727,2722,2716,2711,2705,2700,2695,2689,2684,2679,2673,2668,2663,2657,2652,
    2647,2642,2637,2632,2626,2621,2616,2611,2606,2601,2596,2591,2586,2581,2576,2571,2567,2562,2557,2552,2547,2542,2538,2533,2528,2523,2519,2514,2509,2505,2500,2495,2491,2486,
    2482,2477,2473,2468,2464,2459,2455,2450,2446,2441,2437,2432,2428,2424,2419,2415,2411,2406,2402,2398,2394,2389,2385,2381,2377,2373,2368,2364,2360,2356,2352,2348,2344,2340,
    2336,2332,2328,2324,2320,2316,2312,2308,2304,2300,2296,2292,2288,2284,2280,2277,2273,2269,2265,2261,2258,2254,2250,2246,2243,2239,2235,2231,2228,2224,2220,2217,2213,2209,
    2206,2202,2199,2195,2192,2188,2184,2181,2177,2174,2170,2167,2163,2160,2157,2153,2150,2146,2143,2139,2136,2133,2129,2126,2123,2119,2116,2113,2109,2106,2103,2100,2096,2093,
    2090,2087,2083,2080,2077,2074,2071,2067,2064,2061,2058,2055,2052,2049,2045,2042,2039,2036,2033,2030,2027,2024,2021,2018,2015,2012,2009,2006,2003,2000,1997,1994,1991,1988,
    1985,1982,1979,1977,1974,1971,1968,1965,1962,1959,1957,1954,1951,1948,1945,1942,1940,1937,1934,1931,1929,1926,1923,1920,1918,1915,1912,1909,1907,1904,1901,1899,1896,1893,
    1891,1888,1885,1883,1880,1878,1875,1872,1870,1867,1865,1862,1860,1857,1854,1852,1849,1847,1844,1842,1839,1837,1834,1832,1829,1827,1824,1822,1819,1817,1815,1812,1810,1807,
    1805,1802,1800,1798,1795,1793,1790,1788,1786,1783,1781,1779,1776,1774,1772,1769,1767,1765,1762,1760,1758,1756,1753,1751,1749,1746,1744,1742,1740,1737,1735,1733,1731,1729,
    1726,1724,1722,1720,1718,1715,1713,1711,1709,1707,1705,1702,1700,1698,1696,1694,1692,1690,1688,1685,1683,1681,1679,1677,1675,1673,1671,1669,1667,1665,1663,1661,1658,1656,
    1654,1652,1650,1648,1646,1644,1642,1640,1638,1636,1634,1632,1630,1628,1627,1625,1623,1621,1619,1617,1615,1613,1611,1609,1607,1605,1603,1601,1600,1598,1596,1594,1592,1590,
    1588,1586,1585,1583,1581,1579,1577,1575,1573,1572,1570,1568,1566,1564,1563,1561,1559,1557,1555,1554,1552,1550,1548,1546,1545,1543,1541,1539,1538,1536,1534,1532,1531,1529,
    1527,1525,1524,1522,1520,1519,1517,1515,1513,1512,1510,1508,1507,1505,1503,1502,1500,1498,1497,1495,1493,1492,1490,1488,1487,1485,1484,1482,1480,1479,1477,1475,1474,1472,
    1471,1469,1467,1466,1464,1463,1461,1459,1458,1456,1455,1453,1452,1450,1448,1447,1445,1444,1442,1441,1439,1438,1436,1435,1433,1432,1430,1429,1427,1426,1424,1423,1421,1420,
    1418,1417,1415,1414,1412,1411,1409,1408,1406,1405,1403,1402,1400,1399,1398,1396,1395,1393,1392,1390,1389,1387,1386,1385,1383,1382,1380,1379,1378,1376,1375,1373,1372,1371,
    1369,1368,1366,1365,1364,1362,1361,1360,1358,1357,1355,1354,1353,1351,1350,1349,1347,1346,1345,1343,1342,1341,1339,1338,1337,1335,1334,1333,1331,1330,1329,1327,1326,1325,
    1324,1322,1321,1320,1318,1317,1316,1315,1313,1312,1311,1309,1308,1307,1306,1304,1303,1302,1301,1299,1298,1297,1296,1294,1293,1292,1291,1289,1288,1287,1286,1284,1283,1282,
    1281,1280,1278,1277,1276,1275,1274,1272,1271,1270,1269,1268,1266,1265,1264,1263,1262,1261,1259,1258,1257,1256,1255,1253,1252,1251,1250,1249,1248,1247,1245,1244,1243,1242,
    1241,1240,1239,1237,1236,1235,1234,1233,1232,1231,1230,1228,1227,1226,1225,1224,1223,1222,1221,1220,1218,1217,1216,1215,1214,1213,1212,1211,1210,1209,1208,1206,1205,1204,
    1203,1202,1201,1200,1199,1198,1197,1196,1195,1194,1193,1192,1190,1189,1188,1187,1186,1185,1184,1183,1182,1181,1180,1179,1178,1177,1176,1175,1174,1173,1172,1171,1170,1169,
    1168,1167,1166,1165,1164,1163,1162,1161,1160,1159,1158,1157,1156,1155,1154,1153,1152,1151,1150,1149,1148,1147,1146,1145,1144,1143,1142,1141,1140,1139,1138,1137,1136,1135,
    1134,1134,1133,1132,1131,1130,1129,1128,1127,1126,1125,1124,1123,1122,1121,1120,1119,1118,1118,1117,1116,1115,1114,1113,1112,1111,1110,1109,1108,1107,1107,1106,1105,1104,
    1103,1102,1101,1100,1099,1098,1098,1097,1096,1095,1094,1093,1092,1091,1090,1090,1089,1088,1087,1086,1085,1084,1083,1083,1082,1081,1080,1079,1078,1077,1077,1076,1075,1074,
    1073,1072,1071,1071,1070,1069,1068,1067,1066,1066,1065,1064,1063,1062,1061,1060,1060,1059,1058,1057,1056,1056,1055,1054,1053,1052,1051,1051,1050,1049,1048,1047,1047,1046,
    1045,1044,1043,1042,1042,1041,1040,1039,1038,1038,1037,1036,1035,1034,1034,1033,1032,1031,1031,1030,1029,1028,1027,1027,1026,1025,1024,1024,1023,1022,1021,1020,1020,1019,
    1018,1017,1017,1016,1015,1014,1014,1013,1012,1011,1010,1010,1009,1008,1007,1007,1006,1005,1004,1004,1003,1002,1001,1001,1000,999,999,998,997,996,996,995,994,993,993,992,
    991,990,990,989,988,988,987,986,985,985,984,983,983,982,981,980,980,979,978,978,977,976,975,975,974,973,973,972,971,971,970,969,968,968,967,966,966,965,964,964,963,962,962,
    961,960,959,959,958,957,957,956,955,955,954,953,953,952,951,951,950,949,949,948,947,947,946,945,945,944,943,943,942,941,941,940,939,939,938,938,937,936,936,935,934,934,933,
    932,932,931,930,930,929,928,928,927,927,926,925,925,924,923,923,922,922,921,920,920,919,918,918,917,916,916,915,915,914,913,913,912,912,911,910,910,909,908,908,907,907,906,
    905,905,904,904,903,902,902,901,901,900,899,899,898,898,897,896,896,895,895,894,893,893,892,892,891,891,890,889,889,888,888,887,886,886,885,885,884,884,883,882,882,881,881,
    880,879,879,878,878,877,877,876,875,875,874,874,873,873,872,872,871,870,870,869,869,868,868,867,866,866,865,865,864,864,863,863,862,862,861,860,860,859,859,858,858,857,857,
    856,856,855,854,854,853,853,852,852,851,851,850,850,849,849,848,847,847,846,846,845,845,844,844,843,843,842,842,841,841,840,840,839,839,838,837,837,836,836,835,835,834,834,
    833,833,832,832,831,831,830,830,829,829,828,828,827,827,826,826,825,825,824,824,823,823,822,822,821,821,820,820,819,819,818,818,817,817,816,816,815,815,814,814,813,813,812,
    812,811,811,810,810,809,809,808,808,807,807,806,806,805,805,805,804,804,803,803,802,802,801,801,800,800,799,799,798,798,797,797,796,796,796,795,795,794,794,793,793,792,792,
    791,791,790,790,789,789,789,788,788,787,787,786,786,785,785,784,784,784,783,783,782,782,781,781,780,780,779,779,779,778,778,777,777,776,776,775,775,775,774,774,773,773,772,
    772,771,771,771,770,770,769,769,768,768,767,767,767,766,766,765,765,764,764,764,763,763,762,762,761,761,761,760,760,759,759,758,758,758,757,757,756,756,755,755,755,754,754,
    753,753,753,752,752,751,751,750,750,750,749,749,748,748,748,747,747,746,746,745,745,745,744,744,743,743,743,742,742,741,741,741,740,740,739,739,739,738,738,737,737,736,736,
    736,735,735,734,734,734,733,733,733,732,732,731,731,731,730,730,729,729,729,728,728,727,727,727,726,726,725,725,725,724,724,723,723,723,722,722,722,721,721,720,720,720,719,
    719,718,718,718,717,717,717,716,716,715,715,715,714,714,714,713,713,712,712,712,711,711,711


};

static const uint32_t EINV_TRIP_TIME[1898] =
{
    1.3136e+06,9.8039e+05,7.8049e+05,6.4725e+05,5.5211e+05,4.8077e+05,4.2531e+05,3.8095e+05,3.4468e+05,3.1447e+05,2.8891e+05,2.6702e+05,2.4806e+05,2.3148e+05,2.1686e+05,2.0387e+05,
    1.9226e+05,1.8182e+05,1.7238e+05,1.638e+05,1.5598e+05,1.4881e+05,1.4222e+05,1.3615e+05,1.3053e+05,1.2531e+05,1.2046e+05,1.1594e+05,1.1172e+05,1.0776e+05,1.0405e+05,1.0055e+05,
    97264,94162,91230,88456,85828,83333,80963,78709,76562,74516,72562,70696,68912,67204,65568,64000,62495,61050,59661,58326,57041,55804,54611,53462,52353,51282,50248,49249,48283,47348,
    46444,45568,44720,43898,43101,42328,41578,40850,40143,39456,38788,38139,37508,36894,36296,35714,35148,34596,34058,33535,33024,32526,32040,31566,31103,30651,30210,29780,29359,28948,
    28546,28153,27769,27394,27026,26667,26315,25971,25634,25304,24980,24664,24354,24050,23752,23460,23174,22894,22619,22349,22084,21825,21570,21320,21074,20833,20597,20365,20136,19912,
    19692,19476,19264,19055,18850,18648,18450,18255,18063,17875,17689,17507,17328,17151,16978,16807,16639,16473,16310,16150,15992,15837,15684,15533,15384,15238,15094,14952,14812,14675,
    14539,14405,14273,14143,14015,13889,13764,13642,13521,13401,13284,13167,13053,12940,12829,12719,12610,12503,12398,12293,12190,12089,11989,11890,11792,11696,11601,11507,11414,11322,
    11232,11143,11054,10967,10881,10796,10712,10629,10547,10466,10386,10307,10229,10152,10075,10000,9925,9852,9779,9707,9636,9565,9496,9427,9359,9292,9225,9159,9094,9030,8966,8903,8841,
    8779,8718,8658,8598,8539,8481,8423,8366,8309,8253,8198,8143,8089,8035,7982,7930,7877,7826,7775,7724,7674,7625,7576,7527,7479,7432,7384,7338,7292,7246,7200,7156,7111,7067,7023,6980,
    6937,6895,6853,6811,6770,6729,6689,6649,6609,6570,6531,6492,6454,6416,6378,6341,6304,6268,6231,6195,6160,6124,6089,6055,6020,5986,5952,5919,5886,5853,5820,5788,5756,5724,5692,5661,
    5630,5599,5569,5538,5508,5479,5449,5420,5391,5362,5333,5305,5277,5249,5221,5194,5167,5140,5113,5086,5060,5034,5008,4982,4957,4931,4906,4881,4857,4832,4808,4784,4760,4736,4712,4689,
    4665,4642,4619,4597,4574,4552,4529,4507,4485,4464,4442,4421,4399,4378,4357,4336,4316,4295,4275,4255,4235,4215,4195,4175,4156,4136,4117,4098,4079,4060,4042,4023,4005,3986,3968,3950,
    3932,3914,3897,3879,3862,3845,3827,3810,3793,3776,3760,3743,3727,3710,3694,3678,3662,3646,3630,3614,3598,3583,3567,3552,3537,3522,3507,3492,3477,3462,3447,3433,3418,3404,3390,3375,
    3361,3347,3333,3319,3306,3292,3278,3265,3252,3238,3225,3212,3199,3186,3173,3160,3147,3134,3122,3109,3097,3085,3072,3060,3048,3036,3024,3012,3000,2988,2976,2965,2953,2942,2930,2919,
    2907,2896,2885,2874,2863,2852,2841,2830,2819,2809,2798,2787,2777,2766,2756,2745,2735,2725,2715,2704,2694,2684,2674,2664,2655,2645,2635,2625,2616,2606,2597,2587,2578,2568,2559,2550,
    2540,2531,2522,2513,2504,2495,2486,2477,2468,2460,2451,2442,2434,2425,2417,2408,2400,2391,2383,2374,2366,2358,2350,2342,2333,2325,2317,2309,2301,2294,2286,2278,2270,2262,2255,2247,
    2239,2232,2224,2217,2209,2202,2195,2187,2180,2173,2165,2158,2151,2144,2137,2130,2123,2116,2109,2102,2095,2088,2081,2074,2068,2061,2054,2048,2041,2034,2028,2021,2015,2008,2002,1996,
    1989,1983,1977,1970,1964,1958,1952,1946,1939,1933,1927,1921,1915,1909,1903,1897,1891,1886,1880,1874,1868,1862,1857,1851,1845,1840,1834,1828,1823,1817,1812,1806,1801,1795,1790,1784,
    1779,1774,1768,1763,1758,1753,1747,1742,1737,1732,1727,1721,1716,1711,1706,1701,1696,1691,1686,1681,1676,1672,1667,1662,1657,1652,1647,1643,1638,1633,1628,1624,1619,1614,1610,1605,
    1601,1596,1592,1587,1583,1578,1574,1569,1565,1560,1556,1552,1547,1543,1539,1534,1530,1526,1521,1517,1513,1509,1505,1500,1496,1492,1488,1484,1480,1476,1472,1468,1464,1460,1456,1452,
    1448,1444,1440,1436,1432,1429,1425,1421,1417,1413,1409,1406,1402,1398,1394,1391,1387,1383,1380,1376,1372,1369,1365,1362,1358,1354,1351,1347,1344,1340,1337,1333,1330,1327,1323,1320,
    1316,1313,1309,1306,1303,1299,1296,1293,1289,1286,1283,1280,1276,1273,1270,1267,1263,1260,1257,1254,1251,1248,1244,1241,1238,1235,1232,1229,1226,1223,1220,1217,1214,1211,1208,1205,
    1202,1199,1196,1193,1190,1187,1184,1181,1178,1176,1173,1170,1167,1164,1161,1158,1156,1153,1150,1147,1145,1142,1139,1136,1134,1131,1128,1125,1123,1120,1117,1115,1112,1110,1107,1104,
    1102,1099,1096,1094,1091,1089,1086,1084,1081,1079,1076,1074,1071,1069,1066,1064,1061,1059,1056,1054,1051,1049,1047,1044,1042,1039,1037,1035,1032,1030,1028,1025,1023,1021,1018,1016,
    1014,1011,1009,1007,1005,1002,1000,998,996,993,991,989,987,984,982,980,978,976,974,971,969,967,965,963,961,959,956,954,952,950,948,946,944,942,940,938,936,934,932,930,928,926,924,
    922,920,918,916,914,912,910,908,906,904,902,900,898,896,894,893,891,889,887,885,883,881,879,878,876,874,872,870,868,867,865,863,861,859,858,856,854,852,850,849,847,845,843,842,840,
    838,837,835,833,831,830,828,826,825,823,821,820,818,816,815,813,811,810,808,806,805,803,802,800,798,797,795,794,792,790,789,787,786,784,783,781,779,778,776,775,773,772,770,769,767,
    766,764,763,761,760,758,757,755,754,752,751,749,748,747,745,744,742,741,739,738,737,735,734,732,731,729,728,727,725,724,723,721,720,718,717,716,714,713,712,710,709,708,706,705,704,
    702,701,700,698,697,696,694,693,692,691,689,688,687,685,684,683,682,680,679,678,677,675,674,673,672,670,669,668,667,665,664,663,662,661,659,658,657,656,655,653,652,651,650,649,648,
    646,645,644,643,642,641,639,638,637,636,635,634,633,631,630,629,628,627,626,625,624,623,621,620,619,618,617,616,615,614,613,612,611,610,608,607,606,605,604,603,602,601,600,599,598,
    597,596,595,594,593,592,591,590,589,588,587,586,585,584,583,582,581,580,579,578,577,576,575,574,573,572,571,570,569,568,567,566,565,564,563,562,561,560,559,559,558,557,556,555,554,
    553,552,551,550,549,548,547,547,546,545,544,543,542,541,540,539,538,538,537,536,535,534,533,532,531,531,530,529,528,527,526,525,525,524,523,522,521,520,519,519,518,517,516,515,514,
    514,513,512,511,510,510,509,508,507,506,505,505,504,503,502,501,501,500,499,498,498,497,496,495,494,494,493,492,491,491,490,489,488,487,487,486,485,484,484,483,482,481,481,480,479,
    478,478,477,476,475,475,474,473,473,472,471,470,470,469,468,467,467,466,465,465,464,463,462,462,461,460,460,459,458,458,457,456,456,455,454,453,453,452,451,451,450,449,449,448,447,
    447,446,445,445,444,443,443,442,441,441,440,439,439,438,437,437,436,436,435,434,434,433,432,432,431,430,430,429,429,428,427,427,426,425,425,424,424,423,422,422,421,420,420,419,419,
    418,417,417,416,416,415,414,414,413,413,412,411,411,410,410,409,408,408,407,407,406,406,405,404,404,403,403,402,402,401,400,400,399,399,398,398,397,396,396,395,395,394,394,393,393,
    392,391,391,390,390,389,389,388,388,387,387,386,386,385,384,384,383,383,382,382,381,381,380,380,379,379,378,378,377,377,376,376,375,374,374,373,373,372,372,371,371,370,370,369,369,
    368,368,367,367,366,366,365,365,364,364,363,363,362,362,361,361,361,360,360,359,359,358,358,357,357,356,356,355,355,354,354,353,353,352,352,351,351,351,350,350,349,349,348,348,347,
    347,346,346,345,345,345,344,344,343,343,342,342,341,341,341,340,340,339,339,338,338,337,337,337,336,336,335,335,334,334,334,333,333,332,332,331,331,331,330,330,329,329,328,328,328,
    327,327,326,326,325,325,325,324,324,323,323,323,322,322,321,321,321,320,320,319,319,319,318,318,317,317,316,316,316,315,315,315,314,314,313,313,313,312,312,311,311,311,310,310,309,
    309,309,308,308,308,307,307,306,306,306,305,305,304,304,304,303,303,303,302,302,301,301,301,300,300,300,299,299,299,298,298,297,297,297,296,296,296,295,295,295,294,294,294,293,293,
    292,292,292,291,291,291,290,290,290,289,289,289,288,288,288,287,287,287,286,286,285,285,285,284,284,284,283,283,283,282,282,282,281,281,281,280,280,280,279,279,279,278,278,278,277,
    277,277,276,276,276,275,275,275,275,274,274,274,273,273,273,272,272,272,271,271,271,270,270,270,269,269,269,269,268,268,268,267,267,267,266,266,266,265,265,265,265,264,264,264,263,
    263,263,262,262,262,261,261,261,261,260,260,260,259,259,259,259,258,258,258,257,257,257,256,256,256,256,255,255,255,254,254,254,254,253,253,253,252,252,252,252,251,251,251,250,250,
    250,250,249,249,249,249,248,248,248,247,247,247,247,246,246,246,245,245,245,245,244,244,244,244,243,243,243,243,242,242,242,241,241,241,241,240,240,240,240,239,239,239,239,238,238,
    238,238,237,237,237,236,236,236,236,235,235,235,235,234,234,234,234,233,233,233,233,232,232,232,232,231,231,231,231,230,230,230,230,229,229,229,229,228,228,228,228,227,227,227,227,
    227,226,226,226,226,225,225,225,225,224,224,224,224,223,223,223,223,222,222,222,222,222,221,221,221,221,220,220,220,220,219,219,219,219,219,218,218,218,218,217,217,217,217,216,216,
    216,216,216,215,215,215,215,214,214,214,214,214,213,213,213,213,212,212,212,212,212,211,211,211,211,211,210,210,210,210,209,209,209,209,209,208,208,208,208,208,207,207,207,207,206,
    206,206,206,206,205,205,205,205,205,204,204,204,204,204,203,203,203,203,203,202,202,202,202,202,201,201,201,201,201


};

bool DOR_Init(const TDORSetup* const dorSetup);

void DOR_TimingThread(void* pData);

void DOR_TripThread(void* pData);

#endif

/*!
 * @}
*/
