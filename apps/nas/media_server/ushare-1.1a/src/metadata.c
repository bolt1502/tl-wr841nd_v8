/*
 * metadata.c : GeeXboX uShare CDS Metadata DB.
 * Originally developped for the GeeXboX project.
 * Copyright (C) 2005-2007 Benjamin Zores <ben@geexbox.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
 * GNU Library General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdbool.h>

#include <upnp/upnp.h>
#include <upnp/upnptools.h>

#include "mime.h"
#include "metadata.h"
#include "util_iconv.h"
#include "content.h"
#include "gettext.h"
#include "trace.h"

#define TITLE_UNKNOWN "unknown"

#define MAX_URL_SIZE 32

//#define METADAT_DEBUG
#ifdef METADAT_DEBUG
#define printMetaInfo printf
#else
#define printMetaInfo
#endif

struct upnp_entry_lookup_t {
	int id;
	struct upnp_entry_t *entry_ptr;
};

/* identify the media files num by HouXB, 16Sep10 */
static int gMediaFilsNum = 0;

static char *
getExtension (const char *filename)
{
	char *str = NULL;

	str = strrchr (filename, '.');
	if (str)
		str++;

	return str;
}

static struct mime_type_t *
getMimeType (const char *extension)
{
	extern struct mime_type_t MIME_Type_List[];
	struct mime_type_t *list;
	
	char* mimeExtension = NULL;

	if (!extension)
		return NULL;

	list = MIME_Type_List;
	while (list->extension != INVALID_MIME_EXTENSION)
	{
		mimeExtension = mime_get_extension(list);
		if (NULL == mime_get_extension(list))
			break;
		
		if (!strcasecmp (mimeExtension, extension))
			return list;
		list++;
	}

	return NULL;
}

static bool
is_valid_extension (const char *extension)
{
	if (!extension)
		return false;

	if (getMimeType (extension))
		return true;

	return false;
}

static int
get_list_length (void *list)
{
	void **l = list;
	int n = 0;

	while (*(l++))
		n++;

	return n;
}

static xml_convert_t xml_convert[] = {
	{'"' , "&quot;"},
	{'&' , "&amp;"},
	{'\'', "&apos;"},
	{'<' , "&lt;"},
	{'>' , "&gt;"},
	{'\n', "&#xA;"},
	{'\r', "&#xD;"},
	{'\t', "&#x9;"},
	{0, NULL},
};

static char *
get_xmlconvert (int c)
{
	int j;
	for (j = 0; xml_convert[j].xml; j++)
	{
		if (c == xml_convert[j].charac)
			return xml_convert[j].xml;
	}
	return NULL;
}

static char *
convert_xml (const char *title)
{
	char *newtitle, *s, *t, *xml;
	int nbconvert = 0;

	/* calculate extra size needed */
	for (t = (char*) title; *t; t++)
	{
		xml = get_xmlconvert (*t);
		if (xml)
			nbconvert += strlen (xml) - 1;
	}
	if (!nbconvert)
		return NULL;

	newtitle = s = (char*) malloc (strlen (title) + nbconvert + 1);

	for (t = (char*) title; *t; t++)
	{
		xml = get_xmlconvert (*t);
		if (xml)
		{
			strcpy (s, xml);
			s += strlen (xml);
		}
		else
			*s++ = *t;
	}
	*s = '\0';

	return newtitle;
}

static struct mime_type_t Container_MIME_Type =
	{ -1, UPNP_FOLDER, -1};


static struct upnp_entry_t *
upnp_entry_new (struct ushare_t *ut, const char *name, const char *fullpath,
								struct upnp_entry_t *parent, off_t size, int dir)
{
	struct upnp_entry_t *entry = NULL;
	char *title = NULL, *x = NULL;
	char url_tmp[MAX_URL_SIZE] = { '\0' };
	char *title_or_name = NULL;

	if (!name)
		return NULL;

	/* malloc 1,for entry info by HouXB, 13Sep10 */
	entry = (struct upnp_entry_t *) malloc (sizeof (struct upnp_entry_t));

#ifdef HAVE_DLNA
	entry->dlna_profile = NULL;
	entry->url = NULL;

	/* set valid media file's dlna profile. by HouXB, 28Sep10 */
	if (ut->dlna_enabled && fullpath && !dir)
	{
		/* normal. by HouXB, 08Oct10 */
		/* dlna_profile_t *p = dlna_guess_media_profile (ut->dlna, fullpath);	 */
		/* printf("use old DLNA profile\n"); */
		
		/* special. by HouXB, 08Oct10 */
		dlna_profile_t *p = dlna_guess_media_profile2 (ut->dlna, fullpath);	
		/* printf("use new DLNA profile\n"); */
		
		if (!p)
		{
			free (entry);
			return NULL;
		}
		entry->dlna_profile = p;	

		/* count the media file nums, only the recognized files are effective. by HouXB, 14Apr11 */
		gMediaFilsNum++;
	}
#endif /* HAVE_DLNA */
 
	if (ut->xbox360)
	{
		if (ut->root_entry)
			//entry->id = ut->starting_id + 1 + ut->nr_entries++;
			entry->id = ut->starting_id  + ut->nr_entries++;
		else
			//entry->id = ut->starting_id;
			entry->id = 0; /* Creating the root node so don't use the usual IDs */
	}
	else
		entry->id = ut->starting_id + ut->nr_entries++;
	
	/* 	just save the folder fullpath in entry, 
		leave the item fullpath as NULL, to save memory. 	
		by HouXB, 10Feb12 */
	if (dir)
	{
		entry->fullpath = fullpath ? strdup (fullpath) : NULL;
	}
	else
	{
		entry->fullpath = NULL;
	}
	
	entry->parent = parent;
	entry->child_count =	dir ? 0 : -1;
	entry->title = NULL;

	entry->childs = (struct upnp_entry_t **)malloc (sizeof (struct upnp_entry_t *));
	*(entry->childs) = NULL;

	if (!dir) /* item */
	{
#ifdef HAVE_DLNA
		if (ut->dlna_enabled)
		{
			//entry->mime_type = NULL;
			struct mime_type_t *mime = getMimeType (getExtension (name));
			if (!mime)
			{
				--ut->nr_entries; 
				upnp_entry_free (ut, entry);
				log_error ("Invalid Mime type for %s, entry ignored", name);
				return NULL;
			}
			entry->mime_type = mime;
		}		
		else
		{
#endif /* HAVE_DLNA */
			struct mime_type_t *mime = getMimeType (getExtension (name));
			if (!mime)
			{
				--ut->nr_entries; 
				upnp_entry_free (ut, entry);
				log_error ("Invalid Mime type for %s, entry ignored", name);
				return NULL;
			}
			entry->mime_type = mime;
#ifdef HAVE_DLNA
		}
#endif /* HAVE_DLNA */
			
	if (snprintf (url_tmp, MAX_URL_SIZE, "%d.%s",
							entry->id, getExtension (name)) >= MAX_URL_SIZE)
		log_error ("URL string too long for id %d, truncated!!", entry->id);

		/* Only malloc() what we really need */
		entry->url = strdup (url_tmp);
	}
	else /* container */
	{
		entry->mime_type = &Container_MIME_Type;
		entry->url = NULL;
	}

	/* Try Iconv'ing the name but if it fails the end device
	 may still be able to handle it */
	title = iconv_convert (name);
	if (title)
		title_or_name = title;
	else
	{
		if (ut->override_iconv_err)
		{
			title_or_name = strdup (name);
			log_error ("Entry invalid name id=%d [%s]\n", entry->id, name);
		}
		else
		{
			upnp_entry_free (ut, entry);
			log_error ("Freeing entry invalid name id=%d [%s]\n", entry->id, name);
			return NULL;
		}
	}

	if (!dir)
	{
		x = strrchr (title_or_name, '.');
		if (x)	/* avoid displaying file extension */
			*x = '\0';
	}
	x = convert_xml (title_or_name);
	if (x)
	{
		free (title_or_name);
		title_or_name = x;
	}
	entry->title = title_or_name;

	if (!strcmp (title_or_name, "")) /* DIDL dc:title can't be empty */
	{
		free (title_or_name);
		entry->title = strdup (TITLE_UNKNOWN);
	}

	entry->size = size;
	entry->fd = -1;

	if (entry->id && entry->url)
		log_verbose ("Entry->URL (%d): %s\n", entry->id, entry->url);

	return entry;
}

/* Seperate recursive free() function in order to avoid freeing off
 * the parents child list within the freeing of the first child, as
 * the only entry which is not part of a childs list is the root entry
 */
static void
_upnp_entry_free (struct upnp_entry_t *entry)
{
	struct upnp_entry_t **childs;

	if (!entry)
		return;

	if (entry->fullpath)
		free (entry->fullpath);
	if (entry->title)
		free (entry->title);
	if (entry->url)
		free (entry->url);
#ifdef HAVE_DLNA
	if (entry->dlna_profile)
		entry->dlna_profile = NULL;
#endif /* HAVE_DLNA */

	for (childs = entry->childs; *childs; childs++)
		_upnp_entry_free (*childs);
	free (entry->childs);
}

void
upnp_entry_free (struct ushare_t *ut, struct upnp_entry_t *entry)
{
	if (!ut || !entry)
		return;

	/* Free all entries (i.e. children) */
	if (entry == ut->root_entry)
	{
	struct upnp_entry_t *entry_found = NULL;
	struct upnp_entry_lookup_t *lk = NULL;
	RBLIST *rblist;
	int i = 0;
	
	rblist = rbopenlist (ut->rb);
	lk = (struct upnp_entry_lookup_t *) rbreadlist (rblist);

		while (lk)
		{
			entry_found = lk->entry_ptr;
			if (entry_found)
			{
			 	if (entry_found->fullpath)
			 		free (entry_found->fullpath);
			 	if (entry_found->title)
			 		free (entry_found->title);
			 	if (entry_found->url)
			 		free (entry_found->url);

				free (entry_found);
			 	i++;
			}

			free (lk); /* delete the lookup */
			lk = (struct upnp_entry_lookup_t *) rbreadlist (rblist);
		}

		rbcloselist (rblist);
		rbdestroy (ut->rb);
		ut->rb = NULL;

		log_verbose ("Freed [%d] entries\n", i);
	}
	else
		_upnp_entry_free (entry);

	free (entry);
}

static void
upnp_entry_add_child (struct ushare_t *ut,
											struct upnp_entry_t *entry, struct upnp_entry_t *child)
{
	struct upnp_entry_lookup_t *entry_lookup_ptr = NULL;
	struct upnp_entry_t **childs;
	int n;

	if (!entry || !child)
		return;

	for (childs = entry->childs; *childs; childs++)
	{
		if (*childs == child)
		{
			return;
		}
	}
		
	n = get_list_length ((void *) entry->childs) + 1;
	entry->childs = (struct upnp_entry_t **)realloc (entry->childs, (n + 1) * sizeof (*(entry->childs)));
	entry->childs[n] = NULL;
	entry->childs[n - 1] = child;
	entry->child_count++;

	entry_lookup_ptr = (struct upnp_entry_lookup_t *)malloc (sizeof (struct upnp_entry_lookup_t));
	entry_lookup_ptr->id = child->id;
	entry_lookup_ptr->entry_ptr = child;

	if (rbsearch ((void *) entry_lookup_ptr, ut->rb) == NULL)
		log_info (_("Failed to add the RB lookup tree\n"));
}

struct upnp_entry_t *
upnp_get_entry (struct ushare_t *ut, int id)
{
	struct upnp_entry_lookup_t *res, entry_lookup;

	log_verbose ("Looking for entry id %d\n", id);
	if (id == 0) /* We do not store the root (id 0) as it is not a child */
		return ut->root_entry;

	entry_lookup.id = id;
	res = (struct upnp_entry_lookup_t *)
		rbfind ((void *) &entry_lookup, ut->rb);

	if (res)
	{
		log_verbose ("Found at %p\n",
								 ((struct upnp_entry_lookup_t *) res)->entry_ptr);
		return ((struct upnp_entry_lookup_t *) res)->entry_ptr;
	}

	log_verbose ("Not Found\n");

	return NULL;
}

static void
metadata_add_file (struct ushare_t *ut, struct upnp_entry_t *entry,
									 const char *file, const char *name, struct stat *st_ptr)
{
	if (!entry || !file || !name)
	return;

#ifdef HAVE_DLNA

	/* just test the file which extension is legal.  by HouXB, 30Sep10 */
	if(is_valid_extension(getExtension (file)))
		
/* 	if (ut->dlna_enabled || is_valid_extension (getExtension (file)))by HouXB, 30Sep10 */
		
#else
	if (is_valid_extension (getExtension (file)))
#endif
	{
		struct upnp_entry_t *child = NULL;

		child = upnp_entry_new (ut, name, file, entry, st_ptr->st_size, false);
		if (child)
			upnp_entry_add_child (ut, entry, child);
	}
}

static void
metadata_add_container (struct ushare_t *ut,
						struct upnp_entry_t *entry, const char *container)
{
	struct dirent **namelist;
	int n,i;

	/* restrict the media files nums by HouXB, 16Sep10 */
	if(gMediaFilsNum >= MAX_MEDIA_FILES_NUM)
	{
		printMetaInfo("add container:media files num is too large,"
			" it is greate than the config MAX_MEDIA_FILES_NUM: %d",
			MAX_MEDIA_FILES_NUM);
		return;
	}

	if (!entry || !container)
		return;

	n = scandir (container, &namelist, 0, alphasort);
	if (n < 0)
	{
		perror ("scandir");
		return;
	}

	for (i = 0; i < n; i++)
	{
		struct stat st;
		char *fullpath = NULL;

		/* restrict the media files nums by HouXB, 16Sep10 */
		if (gMediaFilsNum >= MAX_MEDIA_FILES_NUM)
		{		
			printMetaInfo("add file: media files num is too large,"
				" it is greate than the config MAX_MEDIA_FILES_NUM: %d",
				MAX_MEDIA_FILES_NUM);
			break;
		}

		if (namelist[i]->d_name[0] == '.')
		{
			free (namelist[i]);
			continue;
		}

		fullpath = (char *)malloc (strlen (container) + strlen (namelist[i]->d_name) + 2);
		sprintf (fullpath, "%s/%s", container, namelist[i]->d_name);

		log_verbose ("%s\n", fullpath);

		if (stat (fullpath, &st) < 0)
		{
			free (namelist[i]);
			free (fullpath);
			continue;
		}

		if (S_ISDIR (st.st_mode))
		{
			struct upnp_entry_t *child = NULL;

			child = upnp_entry_new (ut, namelist[i]->d_name, fullpath, entry, 0, true);
			if (child)
			{
				metadata_add_container (ut, child, fullpath);
				upnp_entry_add_child (ut, entry, child);
			}
		}
		else /* file item! */
			metadata_add_file (ut, entry, fullpath, namelist[i]->d_name, &st);

		free (namelist[i]);
		free (fullpath);
	}
	
	free (namelist);
}

void
free_metadata_list (struct ushare_t *ut)
{
	ut->init = 0;
	if (ut->root_entry)
		upnp_entry_free (ut, ut->root_entry);
	ut->root_entry = NULL;
	ut->nr_entries = 0;

	/* entry is free, gMediaFilsNum should be reset. by HouXB, 27Sep10 */
	gMediaFilsNum = 0;

	if (ut->rb)
	{
		rbdestroy (ut->rb);
		ut->rb = NULL;
	}

	ut->rb = rbinit (rb_compare, NULL);
	if (!ut->rb)
		log_error (_("Cannot create RB tree for lookups\n"));
}


/******************************************************************************
* FUNCTION      	: folderRelationship()
* AUTHOR        	: HouXB <houxubo@tp-link.net>
* DESCRIPTION  	: get the relationship of two folders 
* INPUT			: @src, folder1's path
*				: @dst, folder2's path 		
*
* OUTPUT        	: 
* RETURN        	: -1, ERROR; 0, euqal; 1, child; 2, parent; 3, no parent-child relation.
* OTHERS        	: 
******************************************************************************/
int folderRelationship(char* src, char* dst)
{
	int i;
	
	if ((NULL == src) ||(NULL == dst))
	{
		return -1;
	}

	if(strlen(src) == strlen(dst))
	{
		for (i=0; i<strlen(src); i++)
		{
			if(src[i] != dst[i])
			{
				return 3;
			}
		}
		return 0;
	}
	else if (strlen(src) > strlen(dst))
	{		
		for (i=0; i<strlen(dst); i++)
		{
			if(src[i] != dst[i])
			{
				return 3;
			}
		}
		return 1;
	}
	else
	{
		for (i=0; i<strlen(src); i++)
		{
			if(src[i] != dst[i])
			{
				return 3;
			}
		}
		return 2;
	}
}

/******************************************************************************
* FUNCTION      	: setScanIndexFlag()
* AUTHOR        	: HouXB <houxubo@tp-link.net>
* DESCRIPTION  	: set scan index flag, the folder will be scan only when the flag was set to 1 
* INPUT			: @scanIndexFlag, flag of folder list index
*
* OUTPUT        	: scanIndexFlag[i] = 0, will be ignore the corresponding folder. 1 will scan
* RETURN        	: 
* OTHERS        	: 
******************************************************************************/
void setScanIndexFlag(int* scanIndexFlag)
{
	int i,j,k;
	
	for (i=0; i<ut->contentlist->count; i++)
	{		
		for (j=0; j<ut->contentlist->count; j++)
		{
			/* mark that this content will be scaned. by HouXB, 14Oct10 */
			scanIndexFlag[i] = 1;
			
			if (i == j)
			{
				continue;
			}
			
			k = folderRelationship(ut->contentlist->content[i], ut->contentlist->content[j]);
			
			/* if a parent folder in the list then ignore this path. by HouXB, 14Oct10 */
			if (1 == k )
			{
				scanIndexFlag[i] = 0;
				break;
			}
			/* if already specify the path then ignore this one. by HouXB, 14Oct10 */
			else if ((j<i) && (1 == scanIndexFlag[j]) && (0 == k) )
			{
				scanIndexFlag[i] = 0;
				break;
			}
		}
	}

	printMetaInfo("scanIndexFlag was set as:\n");
	for (i=0; i<MAX_SHARE_FOLDERS_NUM; i++)
	{
		printMetaInfo("%d\t", scanIndexFlag[i]);
	}
	printMetaInfo("\n");	
}

void
build_metadata_list (struct ushare_t *ut)
{
	int i,j,k;
	int scanIndexFlag[MAX_SHARE_FOLDERS_NUM];
	struct upnp_entry_t *vEntry = NULL;	
		
	static int rebuildIndex = 0;
	
	rebuildIndex++;
	if (rebuildIndex > STARTING_ENTRY_ID_CYCLE)
	{
		rebuildIndex = 0;
	}
	
	ut->starting_id = STARTING_ENTRY_ID_XBOX360 + rebuildIndex*STARTING_ENTRY_ID_STEP;

	log_info (_("Building Metadata List ...\n"));	

	/* build root entry */
	if (!ut->root_entry)
		ut->root_entry = upnp_entry_new (ut, "root", NULL, NULL, -1, true);
	
	vEntry = upnp_entry_new (ut, "MediaServerRoot", NULL, ut->root_entry, -1, true);
	
	upnp_entry_add_child (ut, ut->root_entry, vEntry);

	/* init scanIndex */
	for (i=0; i<MAX_SHARE_FOLDERS_NUM; i++)
	{
		scanIndexFlag[i] = 0;
	}
	
	setScanIndexFlag(scanIndexFlag);

	/* add files from content directory */
	for (i=0 ; i < ut->contentlist->count ; i++)
	{
		if (0 == scanIndexFlag[i])
		{		
			continue;
		}
		
		struct upnp_entry_t *entry = NULL;
		char *title = NULL;
		int size = 0;

		log_info (_("Looking for files in content directory : %s\n"),
							ut->contentlist->content[i]);

		size = strlen (ut->contentlist->content[i]);
		
		if (ut->contentlist->content[i][size - 1] == '/')
			ut->contentlist->content[i][size - 1] = '\0';
		title = strrchr (ut->contentlist->content[i], '/');
		
		if (title)
			title++;
		else
		{
			/* directly use content directory name if no '/' before basename */
			title = ut->contentlist->content[i];
		}

		//entry = upnp_entry_new (ut, title, ut->contentlist->content[i], ut->root_entry, -1, true);
		entry = upnp_entry_new (ut, title, ut->contentlist->content[i], vEntry, -1, true);

		if (!entry)
			continue;
		
		//upnp_entry_add_child (ut, ut->root_entry, entry);
		upnp_entry_add_child (ut, vEntry, entry);
		
		metadata_add_container (ut, entry, ut->contentlist->content[i]);
	}

	log_info (_("Found %d files and subdirectories.\n"), ut->nr_entries);
	ut->init = 1;
}

int
rb_compare (const void *pa, const void *pb,
						const void *config __attribute__ ((unused)))
{
	struct upnp_entry_lookup_t *a, *b;

	a = (struct upnp_entry_lookup_t *) pa;
	b = (struct upnp_entry_lookup_t *) pb;

	if (a->id < b->id)
		return -1;

	if (a->id > b->id)
		return 1;

	return 0;
}

