import cv2
import numpy as np

def calc_cdf(channel):
    numbins = 256
    hist = cv2.calcHist([channel],[0],None,[numbins],(0.,256.))
    #PDF of the reference image (i.e. fractions of all pixels in each bin)
    pdf_hist = hist / channel.size
    #G = (L-1)*CDF(p)
    acc_cdf = pdf_hist.cumsum()
    cdf = np.round((numbins-1)*acc_cdf)
    return cdf

def match_histogram_channel(source, reference):
    g = calc_cdf(reference)
    s = calc_cdf(source)
    f = np.zeros(s.shape,np.uint8)
    for ix_row in xrange(0,s.shape[0]):
        f[ix_row] = np.argmin(np.abs(-g + s[ix_row]))
        
    return f[source]

def match_histogram_bgr(source, reference):
    target = np.zeros_like(source)
    target[:,:,0] = match_histogram_channel(source[:,:,0], reference[:,:,0]) 
    target[:,:,1] = match_histogram_channel(source[:,:,1], reference[:,:,1])
    target[:,:,2] = match_histogram_channel(source[:,:,2], reference[:,:,2])
    return target

def compute_match_score(source, reference):
    target = match_histogram_bgr(source, reference)
    return 1.0 - np.abs((source.astype(np.int32) - target)).mean() / 255.0

def match_table(images):
    scores = np.ones((len(images),len(images)),np.float64)
    for i_image in range(0,len(images)):
        for j_image in range(i_image, len(images)):
            if(i_image != j_image):
                scores[i_image,j_image] = compute_match_score(images[i_image], images[j_image])
    return np.argsort(scores, 0), scores

def match_table_sets(images, num_cameras):
    set_size = len(images) / num_cameras
    sets = []
    for i_set in range(0,num_cameras):
        sets.append(images[set_size*i_set:set_size*(i_set+1)])
    scores = np.ones((num_cameras,num_cameras),np.float64)
    for i_cam in range(0,num_cameras):
        cur_set = sets[i_cam]
        for j_cam in range(i_cam+1,num_cameras):
            other_set = sets[j_cam]
            cur_scores = []
            for i_im in cur_set:
                for j_im in other_set:
                    cur_scores.append(compute_match_score(i_im, j_im))
            score = np.mean(cur_scores)
            scores[i_cam,j_cam] = score
            scores[j_cam,i_cam] = score
    return np.flipud(np.argsort(scores,0)),scores

def best_score_rec(scores, combo, selected, last_score, num_cams, level, num_levels):
    best_score = last_score
    best_combo = combo
    combos_examined = 0
    row_selection = selected.copy()
    if(level < num_levels):
        for i_cam in range(0, num_cams):
            if(i_cam not in selected):
                row_selection.add(i_cam)
                for j_cam in range(i_cam+1, num_cams):
                    if(j_cam not in selected):
                        new_selection = row_selection.copy()
                        new_selection.add(j_cam)
                        new_score = last_score + scores[i_cam,j_cam]
                        score, new_combo, nce = best_score_rec(scores, combo + [(i_cam,j_cam)], new_selection, new_score, num_cams, level+1, num_levels)
                        combos_examined += nce
                        if(score > best_score):
                            best_score = score
                            best_combo = new_combo
    else:
        combos_examined = 1
    return best_score, best_combo, combos_examined    
    
                
def best_pairs(scores, score_func = None):
    processed_scores = scores
    if(score_func != None):
        process = np.vectorize(score_func)
        processed_scores = process(scores)
    num_cams = len(scores)
    num_pairs = num_cams / 2
    return best_score_rec(processed_scores, [], set(), 0.0, num_cams, 0, num_pairs)
                
        
        
    
        
            
    
                
                