#ifndef _LGE_LCD_DEBUG_LOG_H_
#define _LGE_LCD_DEBUG_LOG_H_

#if defined(CONFIG_LGE_LCD_DYNAMIC_LOG)

extern uint32_t lcd_debug_level;

#define lcd_print(level, x...)			\
	do {								\
		if (lcd_debug_level >= (level))	\
			printk(x);					\
	} while (0)

#undef pr_emerg
#undef pr_alert
#undef pr_crit
#undef pr_err
#undef pr_warning
#undef pr_warn
#undef pr_notice
#undef pr_info
#undef pr_debug
#undef pr_devel

#define pr_emerg(x...)					lcd_print(0, x)		/* system is unusable */
#define pr_alert(x...)					lcd_print(1, x)		/* action must be taken immediately */
#define pr_crit(x...)					lcd_print(2, x)		/* critical conditions	*/
#define pr_err(x...)					lcd_print(3, x)		/* error conditions 	*/
#define	pr_warn(x...)					pr_warning(x)
#define pr_warning(x...)				lcd_print(4, x)		/* warning conditions	*/
#define pr_notice(x...)					lcd_print(5, x)		/* normal but significant condition	*/
#define pr_info(x...)					lcd_print(6, x)		/* informational		*/
#define	pr_devel(x...)					pr_debug(x)
#define pr_debug(x...)					pr_info(x)
/*#define pr_devel(x...)				mmc_print(7, x)	*/	/* debug-level message	*/

#endif	/* end of LGE_CHANGE_E */

#endif 	/* _LINUX_DEBUG_LOG_H */
