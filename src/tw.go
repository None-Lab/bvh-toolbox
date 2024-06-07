package tw

import (
	"fmt"
	"io"
	"os"
	"strconv"
	"strings"
)

// BvhNode represents a node in a BVH hierarchy.
type BvhNode struct {
	Value    []string
	Children []*BvhNode
	Parent   *BvhNode
	EndSite  bool
}

// AddChild adds a child node to the current node.
func (n *BvhNode) AddChild(item *BvhNode) {
	item.Parent = n
	n.Children = append(n.Children, item)
}

// Filter returns a list of child nodes whose value matches the given key.
func (n *BvhNode) Filter(key string) []*BvhNode {
	var filtered []*BvhNode
	for _, child := range n.Children {
		if child.Value[0] == key {
			filtered = append(filtered, child)
		}
	}
	return filtered
}

// Get returns the value at the given key index.
func (n *BvhNode) Get(key string) []string {
	for _, child := range n.Children {
		for index, item := range child.Value {
			if item == key {
				if index+1 >= len(child.Value) {
					return nil
				}
				return child.Value[index+1:]
			}
		}
	}
	return nil
}

// Name returns the name of the node.
func (n *BvhNode) Name() string {
	return n.Value[1]
}

// String returns a string representation of the node.
func (n *BvhNode) String() string {
	return strings.Join(n.Value, " ")
}

// Bvh represents a BVH file.
type Bvh struct {
	Data   string
	Root   *BvhNode
	Frames [][]string
}

// Tokenize splits the BVH data into tokens.
func (b *Bvh) Tokenize() {
	firstRound := []string{}
	accumulator := ""
	for _, char := range b.Data {
		if char != '\n' && char != '\r' {
			accumulator += string(char)
		} else if accumulator != "" {
			firstRound = append(firstRound, strings.TrimSpace(accumulator))
			accumulator = ""
		}
	}
	if accumulator != "" {
		firstRound = append(firstRound, strings.TrimSpace(accumulator))
		accumulator = ""
	}
	nodeStack := []*BvhNode{b.Root}
	frameTimeFound := false
	node := (*BvhNode)(nil)
	for _, item := range firstRound {
		parts := strings.Split(item, " ")
		if frameTimeFound {
			b.Frames = append(b.Frames, parts)
			continue
		}
		key := parts[0]
		if key == "{" {
			nodeStack = append(nodeStack, node)
		} else if key == "}" {
			nodeStack = nodeStack[:len(nodeStack)-1]
		} else {
			node = &BvhNode{Value: parts}
			nodeStack[len(nodeStack)-1].AddChild(node)
		}
		if key == "Frame" && parts[1] == "Time:" {
			frameTimeFound = true
		}
	}
}

// Search finds nodes matching the given items.
func (b *Bvh) Search(items ...string) []*BvhNode {
	var foundNodes []*BvhNode
	var checkChildren func(node *BvhNode)
	checkChildren = func(node *BvhNode) {
		if len(node.Value) >= len(items) {
			failed := false
			for index, item := range items {
				if node.Value[index] != item {
					failed = true
					break
				}
			}
			if !failed {
				foundNodes = append(foundNodes, node)
			}
		}
		for _, child := range node.Children {
			checkChildren(child)
		}
	}
	checkChildren(b.Root)
	return foundNodes
}

// GetJoints returns a list of joints in the hierarchy.
func (b *Bvh) GetJoints() []*BvhNode {
	var joints []*BvhNode
	var iterateJoints func(joint *BvhNode)
	iterateJoints = func(joint *BvhNode) {
		joints = append(joints, joint)
		for _, child := range joint.Filter("JOINT") {
			iterateJoints(child)
		}
	}
	iterateJoints(b.Search("ROOT")[0])
	return joints
}

// GetJointsNames returns a list of joint names.
func (b *Bvh) GetJointsNames() []string {
	var joints []string
	var iterateJoints func(joint *BvhNode)
	iterateJoints = func(joint *BvhNode) {
		joints = append(joints, joint.Value[1])
		for _, child := range joint.Filter("JOINT") {
			iterateJoints(child)
		}
	}
	iterateJoints(b.Search("ROOT")[0])
	return joints
}

// JointDirectChildren returns the direct children of a joint.
func (b *Bvh) JointDirectChildren(name string) []*BvhNode {
	joint := b.GetJoint(name)
	return joint.Filter("JOINT")
}

// GetJointIndex returns the index of a joint in the list of joints.
func (b *Bvh) GetJointIndex(name string) int {
	for i, jointName := range b.GetJointsNames() {
		if jointName == name {
			return i
		}
	}
	return -1 // Not found
}

// GetJoint returns the joint with the given name.
func (b *Bvh) GetJoint(name string) *BvhNode {
	found := b.Search("ROOT", name)
	if len(found) == 0 {
		found = b.Search("JOINT", name)
	}
	if len(found) > 0 {
		return found[0]
	}
	return nil
}

// JointOffset returns the offset of a joint.
func (b *Bvh) JointOffset(name string) []float64 {
	joint := b.GetJoint(name)
	offset := joint.Get("OFFSET")
	var result []float64
	for _, item := range offset {
		f, _ := strconv.ParseFloat(item, 64)
		result = append(result, f)
	}
	return result
}

// JointChannels returns the channels of a joint.
func (b *Bvh) JointChannels(name string) []string {
	joint := b.GetJoint(name)
	return joint.Get("CHANNELS")[1:]
}

// GetJointChannelsIndex returns the index of a joint in the list of channels.
func (b *Bvh) GetJointChannelsIndex(jointName string) int {
	index := 0
	for _, joint := range b.GetJoints() {
		if joint.Value[1] == jointName {
			return index
		}
		index += int(joint.Get("CHANNELS")[0][0])
	}
	return -1
}

// GetJointChannelIndex returns the index of a channel in the list of channels.
func (b *Bvh) GetJointChannelIndex(joint string, channel string) int {
	channels := b.JointChannels(joint)
	for i, c := range channels {
		if c == channel {
			return i
		}
	}
	return -1 // Not found
}

// FrameJointChannel returns the value of a channel for a joint at a given frame.
func (b *Bvh) FrameJointChannel(frameIndex int, joint string, channel string, value float64) float64 {
	jointIndex := b.GetJointChannelsIndex(joint)
	channelIndex := b.GetJointChannelIndex(joint, channel)
	if channelIndex == -1 && value != 0.0 {
		return value
	}
	if frameIndex >= len(b.Frames) {
		return 0.0
	}
	if jointIndex+channelIndex >= len(b.Frames[frameIndex]) {
		return 0.0
	}
	f, _ := strconv.ParseFloat(b.Frames[frameIndex][jointIndex+channelIndex], 64)
	return f
}

// FrameJointChannels returns the values of multiple channels for a joint at a given frame.
func (b *Bvh) FrameJointChannels(frameIndex int, joint string, channels []string, value float64) []float64 {
	var values []float64
	jointIndex := b.GetJointChannelsIndex(joint)
	for _, channel := range channels {
		channelIndex := b.GetJointChannelIndex(joint, channel)
		if channelIndex == -1 && value != 0.0 {
			values = append(values, value)
		} else {
			if frameIndex >= len(b.Frames) {
				values = append(values, 0.0)
			} else if jointIndex+channelIndex >= len(b.Frames[frameIndex]) {
				values = append(values, 0.0)
			} else {
				f, _ := strconv.ParseFloat(b.Frames[frameIndex][jointIndex+channelIndex], 64)
				values = append(values, f)
			}
		}
	}
	return values
}

// FramesJointChannels returns the values of multiple channels for a joint across all frames.
func (b *Bvh) FramesJointChannels(joint string, channels []string, value float64) [][]float64 {
	var allFrames [][]float64
	jointIndex := b.GetJointChannelsIndex(joint)
	for _, frame := range b.Frames {
		var values []float64
		for _, channel := range channels {
			channelIndex := b.GetJointChannelIndex(joint, channel)
			if channelIndex == -1 && value != 0.0 {
				values = append(values, value)
			} else {
				if jointIndex+channelIndex >= len(frame) {
					values = append(values, 0.0)
				} else {
					f, _ := strconv.ParseFloat(frame[jointIndex+channelIndex], 64)
					values = append(values, f)
				}
			}
		}
		allFrames = append(allFrames, values)
	}
	return allFrames
}

// JointParent returns the parent of a joint.
func (b *Bvh) JointParent(name string) *BvhNode {
	joint := b.GetJoint(name)
	if joint.Parent == b.Root {
		return nil
	}
	return joint.Parent
}

// JointParentIndex returns the index of the parent of a joint.
func (b *Bvh) JointParentIndex(name string) int {
	joint := b.GetJoint(name)
	if joint.Parent == b.Root {
		return -1
	}
	for i, parentCandidate := range b.GetJoints() {
		if parentCandidate == joint.Parent {
			return i
		}
	}
	return -1 // Should never happen if the parent exists
}

// NFrames returns the number of frames in the BVH file.
func (b *Bvh) NFrames() int {
	framesNode := b.Search("Frames:")[0]
	nFrames, _ := strconv.Atoi(framesNode.Value[1])
	return nFrames
}

// FrameTime returns the frame time of the BVH file.
func (b *Bvh) FrameTime() float64 {
	frameTimeNode := b.Search("Frame", "Time:")[0]
	frameTime, _ := strconv.ParseFloat(frameTimeNode.Value[2], 64)
	return frameTime
}

// BvhTree extends the Bvh class with additional functionality.
type BvhTree struct {
	*Bvh
}

// NewBvhTree creates a new BvhTree from the given BVH data.
func NewBvhTree(data string) *BvhTree {
	bvh := &Bvh{Data: data, Root: &BvhNode{}}
	bvh.Tokenize()
	return &BvhTree{Bvh: bvh}
}

// GetJointsNames returns a list of joint names, including End Sites.
func (bt *BvhTree) GetJointsNames(endSites bool) []string {
	var joints []string
	var iterateJoints func(joint *BvhNode)
	iterateJoints = func(joint *BvhNode) {
		joints = append(joints, joint.Value[1])
		if endSites {
			for _, end := range joint.Filter("End") {
				joints = append(joints, end.Value[1])
			}
		}
		for _, child := range joint.Filter("JOINT") {
			iterateJoints(child)
		}
	}
	iterateJoints(bt.Search("ROOT")[0])
	return joints
}

// GetJoints returns a list of joints, including End Sites.
func (bt *BvhTree) GetJoints(endSites bool) []*BvhNode {
	var joints []*BvhNode
	var iterateJoints func(joint *BvhNode)
	iterateJoints = func(joint *BvhNode) {
		joints = append(joints, joint)
		if endSites {
			for _, end := range joint.Filter("End") {
				joints = append(joints, end)
			}
		}
		for _, child := range joint.Filter("JOINT") {
			iterateJoints(child)
		}
	}
	iterateJoints(bt.Search("ROOT")[0])
	return joints
}

// GetJoint returns the joint with the given name, including End Sites.
func (bt *BvhTree) GetJoint(name string) *BvhNode {
	found := bt.Search("ROOT", name)
	if len(found) == 0 {
		found = bt.Search("JOINT", name)
	}
	if len(found) == 0 {
		found = bt.Search("End", name)
	}
	if len(found) > 0 {
		return found[0]
	}
	return nil
}

// GetJointIndex returns the index of a joint in the list of joints, including End Sites.
func (bt *BvhTree) GetJointIndex(name string) int {
	joint := bt.GetJoint(name)
	if joint.Parent == bt.Root {
		return -1
	}
	for i, parentCandidate := range bt.GetJoints(true) { // Get all joints, including End Sites
		if parentCandidate == joint.Parent {
			return i
		}
	}
	return -1 // Should never happen if the parent exists
}

// JointChildren returns the direct children of a joint, including End Sites.
func (bt *BvhTree) JointChildren(name string) []*BvhNode {
	joint := bt.GetJoint(name)
	var children []*BvhNode
	for _, child := range joint.Filter("JOINT") {
		children = append(children, child)
	}
	for _, child := range joint.Filter("End") {
		children = append(children, child)
	}
	return children
}

// WriteFile writes the BVH data to a file.
func (bt *BvhTree) WriteFile(filePath string) bool {
	file, err := os.Create(filePath)
	if err != nil {
		fmt.Println("ERROR:", err)
		return false
	}
	defer file.Close()
	bt.Write(file)
	return true
}

// Write writes the BVH data to the given stream.
func (bt *BvhTree) Write(out io.Writer) {
	fmt.Fprintf(out, "%s\n", bt.GetHierarchyString())
	fmt.Fprintf(out, "%s\n", bt.GetMotionString())
}

// GetHierarchyString returns the hierarchy string of the BVH data.
func (bt *BvhTree) GetHierarchyString() string {
	var s string
	s += "HIERARCHY\n"
	for _, joint := range bt.GetJoints(true) {
		s = bt.CloseScopes(s, bt.GetJointDepth(joint.Name()))
		s += bt.GetJointString(joint)
	}
	s = bt.CloseScopes(s, 0)
	return s
}

// GetJointDepth returns the depth of a joint in the hierarchy.
func (bt *BvhTree) GetJointDepth(name string) int {
	depth := 0
	parent := bt.JointParent(name)
	for parent != nil {
		depth++
		parent = bt.JointParent(parent.Name())
	}
	return depth
}

// GetJointString returns the string representation of a joint.
func (bt *BvhTree) GetJointString(joint *BvhNode) string {
	depth := bt.GetJointDepth(joint.Name())
	var s string
	if len(bt.JointChildren(joint.Name())) == 0 {
		s += fmt.Sprintf("%sEnd Site\n", strings.Repeat("  ", depth))
		s += fmt.Sprintf("%s{{\n", strings.Repeat("  ", depth))
		s += fmt.Sprintf("%sOFFSET %s\n", strings.Repeat("  ", depth+1), strings.Join(joint.Get("OFFSET"), " "))
		s += fmt.Sprintf("%s}}\n", strings.Repeat("  ", depth))
	} else {
		s += fmt.Sprintf("%s%s\n", strings.Repeat("  ", depth), joint.String())
		s += fmt.Sprintf("%s{{\n", strings.Repeat("  ", depth))
		for _, attribute := range []string{"OFFSET", "CHANNELS"} {
			s += fmt.Sprintf("%s%s %s\n", strings.Repeat("  ", depth+1), attribute, strings.Join(joint.Get(attribute), " "))
		}
	}
	return s
}

// CloseScopes closes open scopes in the hierarchy string.
func (bt *BvhTree) CloseScopes(hierarchyString string, targetDepth int) string {
	lastDepth := strings.Count(strings.Split(hierarchyString, "\n")[len(strings.Split(hierarchyString, "\n"))-2], "  ")
	diff := lastDepth - targetDepth
	for depth := 0; depth < diff; depth++ {
		hierarchyString += fmt.Sprintf("%s}}\n", strings.Repeat("  ", lastDepth-depth-1))
	}
	return hierarchyString
}

// GetMotionString returns the motion string of the BVH data.
func (bt *BvhTree) GetMotionString() string {
	var s string
	s += "MOTION\n"
	s += fmt.Sprintf("Frames: %d\n", bt.NFrames())
	s += fmt.Sprintf("Frame Time: %f\n", bt.FrameTime())
	for _, frame := range bt.Frames {
		s += strings.Join(frame, " ")
		s += "\n"
	}
	return s
}

// contains returns true if the given slice contains the given element.
func contains(s []string, e string) bool {
	for _, a := range s {
		if a == e {
			return true
		}
	}
	return false
}

// index returns the index of the given element in the given slice.
func index(s []string, e string) int {
	for i, a := range s {
		if a == e {
			return i
		}
	}
	return -1
}

func tw() {
	// Example usage:
	bvhData := `HIERARCHY
ROOT Hips
{
  OFFSET 0.0 0.0 0.0
  CHANNELS 6 Xposition Yposition Zposition Xrotation Yrotation Zrotation
  JOINT LeftHip
  {
    OFFSET 0.0 -0.12 0.0
    CHANNELS 3 Xrotation Yrotation Zrotation
    JOINT LeftKnee
    {
      OFFSET 0.0 -0.38 0.0
      CHANNELS 3 Xrotation Yrotation Zrotation
      JOINT LeftAnkle
      {
        OFFSET 0.0 -0.40 0.0
        CHANNELS 3 Xrotation Yrotation Zrotation
        JOINT LeftFoot
        {
          OFFSET 0.0 -0.05 0.0
          CHANNELS 3 Xrotation Yrotation Zrotation
        }
      }
    }
  }
  JOINT RightHip
  {
    OFFSET 0.0 -0.12 0.0
    CHANNELS 3 Xrotation Yrotation Zrotation
    JOINT RightKnee
    {
      OFFSET 0.0 -0.38 0.0
      CHANNELS 3 Xrotation Yrotation Zrotation
      JOINT RightAnkle
      {
        OFFSET 0.0 -0.40 0.0
        CHANNELS 3 Xrotation Yrotation Zrotation
        JOINT RightFoot
        {
          OFFSET 0.0 -0.05 0.0
          CHANNELS 3 Xrotation Yrotation Zrotation
        }
      }
    }
  }
}
MOTION
Frames: 10
Frame Time: 0.0333333
0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
`

	// Create a new BvhTree from the BVH data.
	bvhTree := NewBvhTree(bvhData)

	// Print the joint names, including End Sites.
	fmt.Println("Joint Names:", bvhTree.GetJointsNames(true))

	// Print the number of frames.
	fmt.Println("Number of Frames:", bvhTree.NFrames())

	// Print the frame time.
	fmt.Println("Frame Time:", bvhTree.FrameTime())

	// Get the values of the Xrotation channel for the LeftHip joint for all frames.
	leftHipXRotation := bvhTree.FramesJointChannels("LeftHip", []string{"Xrotation"}, 0.0)
	fmt.Println("LeftHip Xrotation:", leftHipXRotation)

	// Write the BVH data to a file.
	if bvhTree.WriteFile("output.bvh") {
		fmt.Println("BVH data written to output.bvh")
	}
}
