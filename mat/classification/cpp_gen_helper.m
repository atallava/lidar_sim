template = 'ss << "_: " << _ << std::endl;';
stock = '_';

replacements = {'m_section_packet_start','m_section_packet_end','m_section_packet_step', ...
    'm_slice_hit_to_blocks_threshold','m_slice_resn_along_ray', ...
    'm_slice_miss_to_blocks_threshold'};


%%
genPhrasesFromTemplate(template,stock,replacements)
