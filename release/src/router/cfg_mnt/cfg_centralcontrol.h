#ifndef __CFG_CENTRALCONTROL_H__
#define __CFG_CENTRALCONTROL_H__

#define COMMON_CONFIG	"common"

enum ruleAction {
	RULE_ADD = 1,
	RULE_DEL,
	RULE_UPDATE
};

enum followRule {
	FOLLOW_COMMON = 1,
	FOLLOW_PRIVATE
};

extern int cm_updateCommonToPrivateConfig(char *mac, unsigned char *ftList, json_object *cfgRoot);
extern int cm_transformCfgToArray(json_object *cfgObj, json_object *arrayObj);
extern int cm_updatePrivateRuleByMac(char *mac, json_object *cfgObj, int follow, int action);
extern int cm_checkParamFollowRule(char *mac, char *param, int rule);
#ifdef UPDATE_COMMON_CONFIG
extern int cm_updateCommonConfigToFile(json_object *cfgRoot);
#endif
#ifdef RTCONFIG_AMAS_CAP_CONFIG
extern char *cm_getValueFromCommonConfig(const char *name, int needDecrypt);
extern void cm_addConfigFromCommonFileByFeature(json_object *inRoot, json_object *outRoot);
#endif

#endif /* __CFG_CENTRALCONTROL_H__ */
/* End of cfg_centralcontrol.h */